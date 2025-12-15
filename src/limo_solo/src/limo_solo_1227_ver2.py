#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class LineTracerWithObstacleAvoidance:
    def __init__(self):
        rospy.init_node("line_tracer_with_obstacle_avoidance")

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb, queue_size=1)

        self.bridge = CvBridge()

        self.speed = 0.15
        self.scan_ranges = []
        self.front = 999.0

        self.state = "LANE"  # "LANE" / "BACK" / "ESCAPE"
        self.escape_angle = 0.0
        self.state_start = rospy.Time.now().to_sec()

        self.left_escape_count = 0
        self.force_right_escape = 0

        self.robot_width = 0.13

    # ============================================================
    # LIDAR
    # ============================================================
    def lidar_cb(self, scan: LaserScan):
        raw = np.array(scan.ranges, dtype=np.float32)
        self.scan_ranges = raw

        front_zone = np.concatenate([raw[:10], raw[-10:]])
        cleaned = [d for d in front_zone if (d > 0.20 and not np.isnan(d))]
        self.front = float(np.median(cleaned)) if cleaned else 999.0

    # ============================================================
    # CAMERA (라바콘 우선)
    # ============================================================
    def camera_cb(self, msg: CompressedImage):
        now = rospy.Time.now().to_sec()

        if self.state == "ESCAPE":
            self.escape_control()
            return

        if self.state == "BACK":
            self.back_control()
            return

        if self.state == "LANE":
            # 장애물 → BACK
            if self.front < 0.45:
                self.state = "BACK"
                self.state_start = now
                return

            # 카메라 이미지 읽기
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                rospy.logwarn(f"cv_bridge error: {e}")
                return

            h, w = frame.shape[:2]
            roi = frame[int(h * 0.55):h, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # ================================================
            # 1) 라바콘(빨간색) 검출
            # ================================================
            lower_r1 = np.array([0, 120, 80])
            upper_r1 = np.array([10, 255, 255])
            lower_r2 = np.array([170, 120, 80])
            upper_r2 = np.array([180, 255, 255])

            mask_r1 = cv2.inRange(hsv, lower_r1, upper_r1)
            mask_r2 = cv2.inRange(hsv, lower_r2, upper_r2)
            red_mask = cv2.bitwise_or(mask_r1, mask_r2)

            cnts = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours = cnts[0] if len(cnts) == 2 else cnts[1]

            # 라바콘 보이면 → 라인트레이싱 OFF
            if len(red_contours) >= 1:
                twist = Twist()

                centers = []
                for cnt in red_contours:
                    area = cv2.contourArea(cnt)
                    if area < 200:
                        continue
                    M = cv2.moments(cnt)
                    if M["m00"] == 0:
                        continue
                    cx = int(M["m10"] / M["m00"])
                    centers.append(cx)

                if len(centers) == 0:
                    return

                if len(centers) >= 2:
                    centers = sorted(centers)
                    mid = (centers[0] + centers[-1]) // 2
                else:
                    mid = int(centers[0])

                error = mid - (w // 2)
                twist.linear.x = 0.13
                twist.angular.z = error / 180.0
                self.pub.publish(twist)
                return

            # ================================================
            # 2) 라바콘 없으면 → 라인트레이싱 (흰+노란)
            # ================================================
            twist = Twist()

            # --- white line mask ---
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 40, 255])
            white_mask = cv2.inRange(hsv, lower_white, upper_white)

            # --- yellow line mask ---
            # 환경/조명에 따라 튜닝 필요할 수 있음
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([40, 255, 255])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # 흰색 OR 노란색
            mask_line = cv2.bitwise_or(white_mask, yellow_mask)

            cnts2 = cv2.findContours(mask_line, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            contours = cnts2[0] if len(cnts2) == 2 else cnts2[1]

            if len(contours) == 0:
                twist.linear.x = 0.06
                twist.angular.z = 0.4
                self.pub.publish(twist)
                return

            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] == 0:
                return

            cx = int(M["m10"] / M["m00"])
            error = cx - (w // 2)

            twist.linear.x = 0.14
            twist.angular.z = error / 200.0
            self.pub.publish(twist)
            return

    # ============================================================
    # BACK MODE
    # ============================================================
    def back_control(self):
        twist = Twist()
        now = rospy.Time.now().to_sec()

        if now - self.state_start < 1.2:
            twist.linear.x = -0.15
            twist.angular.z = 0.0
            self.pub.publish(twist)
        else:
            angle = self.find_gap_max()
            angle = self.apply_escape_direction_logic(angle)

            self.escape_angle = angle
            self.state = "ESCAPE"
            self.state_start = now

    # ============================================================
    # ESCAPE MODE
    # ============================================================
    def escape_control(self):
        twist = Twist()
        now = rospy.Time.now().to_sec()

        if now - self.state_start < 1.0:
            twist.linear.x = 0.12
            twist.angular.z = self.escape_angle * 1.3
            self.pub.publish(twist)
        else:
            self.state = "LANE"

    def apply_escape_direction_logic(self, angle: float) -> float:
        if self.force_right_escape > 0:
            self.force_right_escape -= 1
            return 0.7

        if angle < 0:
            self.left_escape_count += 1
            if self.left_escape_count >= 4:
                self.force_right_escape = 2
                self.left_escape_count = 0
        else:
            self.left_escape_count = 0

        return angle

    def find_gap_max(self) -> float:
        if len(self.scan_ranges) == 0:
            return 0.0

        raw = np.array(self.scan_ranges, dtype=np.float32)
        ranges = np.concatenate([raw[-60:], raw[:60]])
        ranges = np.where((ranges < 0.20) | np.isnan(ranges), 0.0, ranges)

        idx = int(np.argmax(ranges))
        max_dist = float(ranges[idx])

        if max_dist < (self.robot_width + 0.10):
            return 0.0

        angle_deg = idx - 60
        return float(angle_deg * np.pi / 180.0)


if __name__ == "__main__":
    LineTracerWithObstacleAvoidance()
    rospy.spin()
