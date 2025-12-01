#!/usr/bin/env python3
"""
ROS node that wraps the CNN steering model from ~/CNN_model for real-time
lane following. It subscribes to a camera topic, runs PyTorch inference, and
publishes the predicted steering angle. Optionally it can send commands directly
to the Arduino through the shared SerialBridge.
"""

import os
import sys
from typing import Optional

import cv2
import numpy as np
import rospy
import torch
import torchvision.transforms as T
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import rospkg

from common_serial import SerialBridge

ANGLE_MIN = 45.0
ANGLE_MAX = 135.0


def clamp_angle(value: float) -> float:
    return max(ANGLE_MIN, min(ANGLE_MAX, value))


def load_model(model_path: str, device: torch.device) -> torch.nn.Module:
    """Load SteeringModel with the same heuristics as CNN_model/inference.py."""
    from model import SteeringModel  # type: ignore

    model = SteeringModel().to(device)
    model_ckpt = os.path.expanduser(model_path)

    if not os.path.isfile(model_ckpt):
        raise FileNotFoundError(model_ckpt)

    try:
        state = torch.load(model_ckpt, map_location=device, weights_only=True)
    except TypeError:
        state = torch.load(model_ckpt, map_location=device)

    if isinstance(state, dict) and "state_dict" in state:
        state = state["state_dict"]
    elif isinstance(state, dict) and "model_state" in state:
        state = state["model_state"]

    model.load_state_dict(state)
    model.eval()
    return model


class LaneCnnNode:
    def __init__(self) -> None:
        rospy.init_node("lane_cnn_node")
        self.bridge = CvBridge()

        pkg_path = rospkg.RosPack().get_path("perception_lane_cnn")
        default_model_root = os.path.join(pkg_path, "cnn_model")
        model_root = os.path.expanduser(
            rospy.get_param("~model_root", default_model_root)
        )
        if model_root not in sys.path:
            sys.path.insert(0, model_root)

        default_weights = os.path.join(pkg_path, "weights", "best_model.pth")
        weights_path = rospy.get_param("~model_path", default_weights)
        use_cuda = rospy.get_param("~use_cuda", True) and torch.cuda.is_available()
        self.device = torch.device("cuda:0" if use_cuda else "cpu")
        rospy.loginfo("Lane CNN using device: %s", self.device)

        self.model = load_model(weights_path, self.device)

        self.transform = T.Compose(
            [
                T.Resize((180, 320)),
                T.Lambda(lambda img: img.crop((0, 120, 320, 180))),
                T.ToTensor(),
                T.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5]),
            ]
        )

        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.pub_topic = rospy.get_param("~steering_topic", "/lane/steering_cnn")
        self.pub = rospy.Publisher(self.pub_topic, Float32, queue_size=10)

        self.frame_skip = max(0, rospy.get_param("~frame_skip", 0))
        self._frame_count = 0
        self.serial_enabled = rospy.get_param("~serial_output", False)
        self.serial_speed = int(rospy.get_param("~serial_speed", 97))
        self.serial: Optional[SerialBridge] = None
        if self.serial_enabled:
            port = rospy.get_param("~serial_port", "/dev/arduino")
            baud = int(rospy.get_param("~serial_baud", 115200))
            self.serial = SerialBridge(port=port, baudrate=baud)

        self.camera_sub = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )
        rospy.loginfo(
            "lane_cnn_node ready: camera=%s, publish=%s, weights=%s",
            self.camera_topic,
            self.pub_topic,
            weights_path,
        )

    # ----------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        self._frame_count += 1
        if self.frame_skip and (self._frame_count % (self.frame_skip + 1)) != 1:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("CvBridge error: %s", exc)
            return

        steering = self.predict(frame)
        self.pub.publish(Float32(data=steering))
        if self.serial_enabled and self.serial:
            try:
                self.serial.send(int(round(steering)), self.serial_speed)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(5.0, "Serial send failed: %s", exc)

    # ----------------------------------------------------------------------
    def predict(self, frame_bgr: np.ndarray) -> float:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        tensor = self.transform(pil).unsqueeze(0).to(self.device)
        with torch.no_grad():
            angle = self.model(tensor).item()
        return float(clamp_angle(angle))


def main() -> None:
    try:
        LaneCnnNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
