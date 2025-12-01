#!/usr/bin/env python3
"""
Convert LaserScan data into actionable distance summaries and recommended
steering/speed commands. Adapts the previous prac_src/rplidar_ros logic but
publishes a structured message for the fusion node. Optional serial output to
Arduino is preserved for standalone testing.
"""

import math
import time
from typing import Optional

import rospy
from sensor_msgs.msg import LaserScan

from common_serial import SerialBridge, create_command
from perception_lidar.msg import LidarState

CM = 100.0


class LidarSupervisor:
    def __init__(self) -> None:
        rospy.init_node("lidar_state_node")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.publisher = rospy.Publisher("lidar/state", LidarState, queue_size=10)

        # Speed profile (use the same semantics as the Arduino firmware: 90=stop)
        self.speed_stop = int(rospy.get_param("~speed_stop", 90))
        self.speed_reverse = int(rospy.get_param("~speed_reverse", 70))
        self.speed_slow = int(rospy.get_param("~speed_slow", 97))
        self.speed_medium = int(rospy.get_param("~speed_medium", 98))
        self.speed_normal = int(rospy.get_param("~speed_normal", 99))

        self.serial_enabled = rospy.get_param("~serial_output", False)
        self.serial: Optional[SerialBridge] = None
        if self.serial_enabled:
            port = rospy.get_param("~serial_port", "/dev/arduino")
            baud = int(rospy.get_param("~serial_baud", 115200))
            self.serial = SerialBridge(port=port, baudrate=baud)

        self.scan_sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self.scan_callback, queue_size=1
        )
        rospy.loginfo("perception_lidar listening on %s", self.scan_topic)

    # ------------------------------------------------------------------
    def scan_callback(self, scan: LaserScan) -> None:
        front = self._sector(scan, 180, 40)
        front_right = self._sector(scan, 135, 30)
        right = self._sector(scan, 90, 30)
        front_left = self._sector(scan, -135, 30)
        left = self._sector(scan, -90, 30)
        rear = self._sector(scan, 0, 40)

        steering = 90
        speed = self.speed_normal
        action = "직진"

        if front < 30:
            action = "긴급 회피"
            self._send_sequence([
                (90, self.speed_stop, 0.2),
                (90, self.speed_reverse, 0.5),
                (90, self.speed_stop, 0.2),
            ])
            if left > right + 20:
                steering = 45
                action = "긴급 좌회전"
            else:
                steering = 135
                action = "긴급 우회전"
            speed = self.speed_slow

        elif front < 50:
            if front_left > front_right + 10:
                steering = 45
                action = "좌회전"
            else:
                steering = 135
                action = "우회전"
            speed = self.speed_slow

        elif front < 100:
            if left < 40:
                steering = 60
                action = "우측 회피"
            elif right < 40:
                steering = 120
                action = "좌측 회피"
            elif front_left > front_right + 20:
                steering = 110
                action = "좌측 편향"
            elif front_right > front_left + 20:
                steering = 70
                action = "우측 편향"
            else:
                steering = 90
                action = "직진 (주의)"
            speed = self.speed_medium

        elif left < 40:
            steering = 70
            action = "좌측 벽 회피"
            speed = self.speed_normal
        elif right < 40:
            steering = 110
            action = "우측 벽 회피"
            speed = self.speed_normal

        state = LidarState()
        state.header.stamp = rospy.Time.now()
        state.front = front
        state.front_right = front_right
        state.right = right
        state.front_left = front_left
        state.left = left
        state.rear = rear
        state.recommended_speed = float(speed)
        state.recommended_steering = int(steering)
        state.action = action
        self.publisher.publish(state)

        rospy.loginfo_throttle(
            0.5,
            "전:%5.1f 전우:%5.1f 우:%5.1f 전좌:%5.1f 좌:%5.1f | %s | 조향:%3d 속도:%3d",
            front,
            front_right,
            right,
            front_left,
            left,
            action,
            steering,
            speed,
        )

        if self.serial_enabled and self.serial:
            try:
                self.serial.send(steering, speed)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(5.0, "Serial send failed: %s", exc)

    # ------------------------------------------------------------------
    def _sector(self, scan: LaserScan, center_deg: float, width_deg: float) -> float:
        minimum = 200.0  # default 2m
        for idx, distance in enumerate(scan.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle_deg = math.degrees(scan.angle_min + idx * scan.angle_increment)
            diff = abs(angle_deg - center_deg)
            if diff > 180:
                diff = 360 - diff
            if diff <= width_deg / 2.0:
                minimum = min(minimum, distance * CM)
        return minimum

    def _send_sequence(self, sequence) -> None:
        if not (self.serial_enabled and self.serial):
            return
        for steering, speed, delay in sequence:
            try:
                self.serial.send(steering, speed)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("Serial sequence failed: %s", exc)
                break
            time.sleep(delay)


def main() -> None:
    try:
        LidarSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
