#!/usr/bin/env python3
"""
Lightweight camera publisher using OpenCV VideoCapture.
Publishes sensor_msgs/Image on /camera/image_raw for the lane CNN and YOLO nodes.
"""

import argparse
import glob
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--topic", default="/camera/image_raw")
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    device = args.device
    candidates = [device] + [d for d in sorted(glob.glob("/dev/video*")) if d != device]
    cap = None
    while not rospy.is_shutdown():
        for dev in candidates:
            rospy.loginfo("Trying camera: %s", dev)
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                device = dev
                break
        if cap and cap.isOpened():
            break
        rospy.logwarn("Failed to open any camera. Retrying in 1s. Tried: %s", candidates)
        time.sleep(1.0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    rospy.init_node("simple_cam_pub")
    pub = rospy.Publisher(args.topic, Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(args.fps or 30)
    rospy.loginfo("Publishing camera %s to %s (%dx%d @%dfps)", args.device, args.topic, args.width, args.height, args.fps)

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            rospy.logwarn_throttle(5.0, "Camera read failed")
            time.sleep(0.01)
            continue
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
