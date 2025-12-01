#!/usr/bin/env python3
"""
ROS node that wraps a YOLOv8 detector (ultralytics) and publishes detection results.
Keeps the ROS message contract unchanged so downstream nodes stay compatible.
"""

import os
from typing import Dict, List, Optional

# YOLOv8 내부에서 YOLOv5 체크 유틸을 호출할 때 불필요한 pip 자동설치를 막기 위함
os.environ["YOLOv5_AUTOINSTALL"] = "0"

import ast
import rospy
import torch
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

from perception_object.msg import DetectionArray, ObjectDetection

# Limit CPU thread usage to avoid saturating all cores
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
cv2.setNumThreads(0)


class ObjectDetectorNode:
    def __init__(self) -> None:
        rospy.init_node("object_yolo_node")
        self.bridge = CvBridge()

        weights = rospy.get_param("~weights", "/home/huins/ai_ws/models/best.pt")
        self.img_size = int(rospy.get_param("~img_size", 512))
        self.conf_thres = float(rospy.get_param("~conf_threshold", 0.50))
        self.iou_thres = float(rospy.get_param("~iou_threshold", 0.45))
        self.use_fp16 = bool(rospy.get_param("~use_fp16", True))
        self.device = (
            "cuda:0" if torch.cuda.is_available() and rospy.get_param("~use_cuda", True) else "cpu"
        )

        self.target_labels: Optional[List[str]] = None
        labels_param = rospy.get_param("~target_labels", [])
        if isinstance(labels_param, str):
            try:
                parsed = ast.literal_eval(labels_param)
                if isinstance(parsed, list):
                    labels_param = parsed
            except Exception:
                pass
        if isinstance(labels_param, list) and labels_param:
            self.target_labels = [str(x).lower() for x in labels_param]

        self.model = self._load_model(weights, self.use_fp16 and self.device.startswith("cuda"))
        rospy.loginfo(
            "Loaded YOLO model: %s (conf=%.2f, iou=%.2f, device=%s, fp16=%s, imgsz=%d)",
            weights,
            self.conf_thres,
            self.iou_thres,
            self.device,
            "on" if self.use_fp16 and self.device.startswith("cuda") else "off",
            self.img_size,
        )

        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.pub_topic = rospy.get_param("~detections_topic", "/detections/yolo")
        self.publisher = rospy.Publisher(self.pub_topic, DetectionArray, queue_size=10)
        self.subscriber = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if not getattr(self, "_dumped_frame", False):
            try:
                cv2.imwrite("/tmp/yolo_cb.jpg", frame)
                rospy.loginfo("Saved debug frame to /tmp/yolo_cb.jpg")
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("Failed to dump debug frame: %s", exc)
            self._dumped_frame = True
        results = self.model(
            frame,
            imgsz=self.img_size,
            conf=self.conf_thres,
            iou=self.iou_thres,
            device=self.device,
            verbose=False,
        )

        detections = DetectionArray()
        detections.header = msg.header

        if not results:
            rospy.loginfo_throttle(2.0, "YOLO raw=0 filtered=0 (no results)")
            self.publisher.publish(detections)
            return

        r = results[0]
        names = self._names_dict(r.names if hasattr(r, "names") else self.model.names)
        target_ids = None
        if self.target_labels:
            target_ids = {
                idx
                for idx, label in names.items()  # type: ignore[arg-type]
                if str(label).lower() in self.target_labels
            }

        if r.boxes is None or len(r.boxes) == 0:
            if not getattr(self, "_dumped_empty", False):
                try:
                    cv2.imwrite("/tmp/yolo_empty.jpg", frame)
                    rospy.logwarn(
                        "Saved empty-detection frame to /tmp/yolo_empty.jpg (shape=%s, mean=%.1f)",
                        frame.shape,
                        float(frame.mean()),
                    )
                except Exception as exc:  # noqa: BLE001
                    rospy.logwarn("Failed to dump empty-detection frame: %s", exc)
                self._dumped_empty = True
            rospy.loginfo_throttle(
                2.0,
                "YOLO raw=0 filtered=0 (target_labels=%s, device=%s, conf>=%.2f)",
                self.target_labels if self.target_labels is not None else "none",
                self.device,
                self.conf_thres,
            )
            self.publisher.publish(detections)
            return

        raw_count = len(r.boxes)
        max_conf = max(float(b.conf.item()) for b in r.boxes) if raw_count else 0.0

        for box in r.boxes:
            cls_id = int(box.cls.item())
            if target_ids is not None and cls_id not in target_ids:
                continue
            x1, y1, x2, y2 = [float(x) for x in box.xyxy[0].tolist()]
            detection = ObjectDetection()
            detection.class_id = cls_id
            detection.label = names.get(cls_id, str(cls_id))
            detection.confidence = float(box.conf.item())
            detection.xmin = x1
            detection.ymin = y1
            detection.xmax = x2
            detection.ymax = y2
            detections.detections.append(detection)

        if detections.detections:
            top = max(detections.detections, key=lambda d: d.confidence)
            rospy.loginfo_throttle(
                2.0,
                "YOLO raw=%d filtered=%d (top %.2f %s, max_conf_raw=%.2f, device=%s)",
                raw_count,
                len(detections.detections),
                top.confidence,
                top.label,
                max_conf,
                self.device,
            )
        else:
            rospy.loginfo_throttle(
                2.0,
                "YOLO raw=%d filtered=0 (target_labels=%s, max_conf_raw=%.2f, device=%s)",
                raw_count,
                self.target_labels if self.target_labels is not None else "none",
                max_conf,
                self.device,
            )
        self.publisher.publish(detections)

    # ------------------------------------------------------------------
    def _load_model(self, weights: str, use_fp16: bool):
        """Load YOLOv8 model from local weights path."""
        if not os.path.isfile(weights):
            raise FileNotFoundError(f"YOLO weights not found: {weights}")

        try:
            model = YOLO(weights)
            model.fuse()  # speed up if supported
            model.to(self.device)
            if use_fp16:
                model.model.half()
            model.eval()
            return model
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("Failed to load YOLOv8 model: %s", exc)
            raise

    @staticmethod
    def _names_dict(names: Optional[Dict[int, str]]):
        """Normalize names list/dict from ultralytics result/model into a dict."""
        if names is None:
            return {}
        if isinstance(names, dict):
            return names
        if isinstance(names, list):
            return {i: n for i, n in enumerate(names)}
        return {}


def main() -> None:
    try:
        ObjectDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
