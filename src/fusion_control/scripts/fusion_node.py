#!/usr/bin/env python3
"""
High-level fusion controller. It combines lane steering (from the CNN node),
LiDAR safety margins, and YOLO detections to produce a final steering/speed
command. The output is both published as a Twist for logging and, optionally,
sent to the Arduino over SerialBridge.
"""

from __future__ import annotations

import math
import ast
from typing import List, Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from common_serial import SerialBridge
from perception_lidar.msg import LidarState
from perception_object.msg import DetectionArray


class FusionController:
    def __init__(self) -> None:
        rospy.init_node("fusion_node")

        self.default_speed = int(rospy.get_param("~default_speed", 99))   # 주행 기본값
        self.max_speed = int(rospy.get_param("~max_speed", 99))          # 상한
        self.min_speed = int(rospy.get_param("~min_speed", 94))          # 하한
        # ESC가 멈추는 값이 94 부근이므로 그 아래로 떨어뜨리지 않도록 고정
        self.stop_speed = max(94, int(rospy.get_param("~stop_speed", 94)))
        self.min_moving_speed = float(rospy.get_param("~min_moving_speed", 99.0))  # ESC가 실제로 굴러가기 시작하는 최소값
        self.steering_slowdown_gain = float(rospy.get_param("~steering_slowdown_gain", 0.0))  # 코너에서도 속도 유지
        self.corner_speed_boost = float(rospy.get_param("~corner_speed_boost", 0.0))  # 코너 시 추가 보정 없음
        self.corner_boost_threshold = float(rospy.get_param("~corner_boost_threshold", 10.0))  # 사용 안 함
        self.speed_slew_rate = float(rospy.get_param("~speed_slew_rate", 5.0))  # 주기당 속도 변화 제한(단위: PWM 값)
        self.steering_center_offset = float(rospy.get_param("~steering_center_offset", 0.0))  # 하드웨어 직진 오프셋(°)
        self.steering_slew_rate = float(rospy.get_param("~steering_slew_rate", 3.0))  # 주기당 조향 변화 제한
        self.steer_from_cnn_only = rospy.get_param("~steer_from_cnn_only", True)  # 조향은 CNN 우선, LiDAR는 속도만 제한
        self.serial_enabled = rospy.get_param("~serial_output", True)
        self.serial_port = rospy.get_param("~serial_port", "/dev/arduino")
        self.serial_baud = int(rospy.get_param("~serial_baud", 115200))

        self.stop_labels: List[str] = [label.lower() for label in self._parse_list_param("~stop_labels", ["person", "stop_sign", "red"])]
        # 정지 감지 민감도: 기본 0.5 이상만 정지 트리거
        self.stop_conf = float(rospy.get_param("~stop_confidence", 0.5))
        self.stop_hold = rospy.Duration(rospy.get_param("~stop_hold", 1.5))
        self.front_stop_distance = float(rospy.get_param("~front_stop_distance", 40.0))
        self.lidar_speed_limit_distance = float(rospy.get_param("~lidar_speed_limit_distance", 80.0))

        self.lane_topic = rospy.get_param("~lane_topic", "/lane/steering_cnn")
        self.lidar_topic = rospy.get_param("~lidar_topic", "lidar/state")
        self.detection_topic = rospy.get_param("~detection_topic", "/detections/yolo")
        self.command_topic = rospy.get_param("~command_topic", "/fusion/command")

        self.lane_angle = 90.0
        self.lidar_state: Optional[LidarState] = None
        self.stop_until = rospy.Time(0)
        self.warmup_lane_count = 0
        self.warmup_yolo_count = 0
        self.warmup_needed = int(rospy.get_param("~warmup_messages", 1))
        self.warmup_require_yolo = rospy.get_param("~warmup_require_yolo", False)
        self.warmup_done = False
        self.smoothed_speed = float(self.default_speed)
        self.smoothed_steering = 90.0

        self.cmd_pub = rospy.Publisher(self.command_topic, Twist, queue_size=10)
        rospy.Subscriber(self.lane_topic, Float32, self.lane_callback, queue_size=1)
        rospy.Subscriber(self.lidar_topic, LidarState, self.lidar_callback, queue_size=1)
        rospy.Subscriber(self.detection_topic, DetectionArray, self.detection_callback, queue_size=1)

        self.serial = SerialBridge(port=self.serial_port, baudrate=self.serial_baud) if self.serial_enabled else None

        control_rate = rospy.get_param("~control_rate", 25.0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / control_rate), self.control_step)
        rospy.loginfo("fusion_node started (rate=%.1fHz)", control_rate)

    def publish_command(self, steering: float, speed: float) -> None:
        steering_int = int(round(steering))
        speed_int = int(round(speed))

        twist = Twist()
        twist.linear.x = (speed_int - 90) / 10.0
        twist.angular.z = -(steering_int - 90) / 45.0
        self.cmd_pub.publish(twist)

        if self.serial_enabled and self.serial:
            try:
                self.serial.send(steering_int, speed_int)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(
                    5.0, "Serial send failed: %s (steer=%s speed=%s)", exc, steering_int, speed_int
                )

    # ------------------------------------------------------------------
    @staticmethod
    def _parse_list_param(name: str, default: List[str]) -> List[str]:
        """
        Robustly parse list params even when they are passed as YAML-looking strings
        like "['person','stop_sign']". If parsing fails, fall back to default.
        """
        value = rospy.get_param(name, default)
        if isinstance(value, list):
            return value
        if isinstance(value, str):
            try:
                parsed = ast.literal_eval(value)
                if isinstance(parsed, list):
                    return parsed
            except Exception:
                pass
        rospy.logwarn("Param %s is not a list; using default %s", name, default)
        return default

    def lane_callback(self, msg: Float32) -> None:
        self.lane_angle = float(msg.data)
        if not self.warmup_done:
            self.warmup_lane_count += 1
            self._check_warmup()

    def lidar_callback(self, msg: LidarState) -> None:
        self.lidar_state = msg

    def detection_callback(self, msg: DetectionArray) -> None:
        now = rospy.Time.now()
        for det in msg.detections:
            label = det.label.lower()
            conf = float(det.confidence)
            if label in self.stop_labels:
                if conf < self.stop_conf:
                    rospy.loginfo_throttle(
                        1.0, "Stop ignored (conf %.2f < %.2f): %s", conf, self.stop_conf, det.label
                    )
                    continue
                self.stop_until = max(self.stop_until, now + self.stop_hold)
                # 정지 이벤트 즉시 반영: 속도 버퍼를 낮추고 바로 전송
                self.smoothed_speed = float(self.stop_speed)
                self.publish_command(self.smoothed_steering, float(self.stop_speed))
                rospy.loginfo_throttle(1.0, "Stop trigger: %s (%.2f)", det.label, conf)
        if not self.warmup_done and msg.detections:
            self.warmup_yolo_count += 1
            self._check_warmup()

    def _check_warmup(self) -> None:
        if self.warmup_done:
            return
        lane_ready = self.warmup_lane_count >= self.warmup_needed
        yolo_ready = (not self.warmup_require_yolo) or (self.warmup_yolo_count >= self.warmup_needed)
        if lane_ready and yolo_ready:
            self.warmup_done = True

    # ------------------------------------------------------------------
    def control_step(self, event) -> None:
        steering = self.lane_angle + self.steering_center_offset
        target_speed = float(self.default_speed)

        if not self.warmup_done:
            target_speed = self.stop_speed
            steering = 90.0
            if rospy.get_time() % 1.0 < 0.05:
                rospy.loginfo_throttle(1.0, "Waiting for warmup: lane=%d yolo=%d/%d", self.warmup_lane_count, self.warmup_yolo_count, self.warmup_needed)
            self.publish_command(steering, target_speed)
            return

        if self.lidar_state:
            front = self.lidar_state.front
            # 조향은 CNN 우선, 속도만 LiDAR로 제한 (필요 시 조향도 덮어쓰려면 steer_from_cnn_only=False)
            if not self.steer_from_cnn_only and front < self.front_stop_distance:
                steering = self.lidar_state.recommended_steering
            # 충분히 멀리 앞이 비었을 때는 LiDAR 속도 제한을 적용하지 않음
            if front <= self.lidar_speed_limit_distance:
                target_speed = min(target_speed, float(self.lidar_state.recommended_speed))

        stop_active = rospy.Time.now() < self.stop_until

        # 조향량에 따라 미세 감속(좌우 꺾을 때 속도가 약간 줄도록)
        steering = max(45.0, min(135.0, steering))
        steer_offset = abs(steering - 90.0)
        if stop_active:
            # 정지 이벤트 시에는 최소속도/램프/하한을 무시하고 바로 정지 값을 보냄
            target_speed = float(self.stop_speed)
            target_speed = max(0.0, min(180.0, target_speed))
            self.smoothed_speed = target_speed
        else:
            target_speed -= self.steering_slowdown_gain * steer_offset
            # 코너에서는 감속 보정으로 속도를 +1 정도 유지
            if steer_offset >= self.corner_boost_threshold:
                target_speed += self.corner_speed_boost

            # ESC가 실제로 굴러가기 시작하는 하한 보정
            if target_speed > 90.0 and target_speed < self.min_moving_speed:
                target_speed = self.min_moving_speed

            # 범위 클램프
            target_speed = max(float(self.min_speed), min(float(self.max_speed), target_speed))

            # 속도 램프(가속/감속 단계 제한)
            delta = target_speed - self.smoothed_speed
            step = self.speed_slew_rate
            if abs(delta) > step:
                delta = math.copysign(step, delta)
            self.smoothed_speed += delta

        # 조향 램프(급격한 조향 변화 제한)
        steer_delta = steering - self.smoothed_steering
        max_steer_step = self.steering_slew_rate
        if abs(steer_delta) > max_steer_step:
            steer_delta = math.copysign(max_steer_step, steer_delta)
        self.smoothed_steering += steer_delta

        steering_int = int(round(self.smoothed_steering))
        target_speed_int = int(round(self.smoothed_speed))

        twist = Twist()
        twist.linear.x = (target_speed_int - 90) / 10.0
        twist.angular.z = -(steering_int - 90) / 45.0
        self.cmd_pub.publish(twist)

        if self.serial_enabled and self.serial:
            try:
                self.serial.send(steering_int, target_speed_int)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(
                    5.0, "Serial send failed: %s (steer=%s speed=%s)", exc, steering_int, target_speed_int
                )


def main() -> None:
    try:
        FusionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
