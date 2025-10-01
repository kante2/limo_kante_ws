#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
LiDAR ROI를 나눠 포인트 시각화(cv2 창) + 장애물 정보 퍼블리시
"""

import os, sys
import rospy
import json
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class PerLidar:
    def __init__(self):
        print("PerLidar start")
        rospy.init_node('per_lidar_node')

        # ===== Viz params =====
        self.show_rviz   = bool(rospy.get_param("~show_rviz", True))
        self.win_name   = rospy.get_param("~window_name", "Lidar View")
        self.img_size   = int(rospy.get_param("~img_size", 500))  # 픽셀
        self.scale      = float(rospy.get_param("~scale", 25.0))  # m -> 픽셀 변환 (값 ↑ = 확대)

        self.init_pubSub()
        self.init_msg()
        self.init_ROI()
        if self.show_rviz:
            self.init_rviz()

        rospy.on_shutdown(self.on_shutdown)

    # ---------------- init blocks ----------------
    def init_pubSub(self):
        scan_topic = rospy.get_param("~scan_topic", "/limo/scan")
        rospy.Subscriber(scan_topic, LaserScan, self.CB_lidar_raw, queue_size=1)
        self.pub_lidar = rospy.Publisher('/perception/lidar', String, queue_size=1)
        rospy.loginfo(f"[PerLidar] subscribe: {scan_topic}")

    def init_msg(self):
        self.laser_data = None

    def init_ROI(self):
        # 너가 쓰던 값 그대로 유지
        self.length_x_front = -0.8
        self.length_y_front = 0.2
        self.length_x_front_rotary = -0.6
        self.length_x_front_near = -1.2
        self.length_y_front_near = 0.05
        self.length_x_left = -0.8
        self.length_y_left_min = -0.325
        self.length_y_left_max = -0.175
        self.length_x_right = -1.0
        self.length_y_right_min = 0.175
        self.length_y_right_max = 0.55
        self.length_x_rotary = -1.3
        self.length_y_rotary_min = -0.7
        self.length_y_rotary_max = 0.175

    def init_rviz(self):
        # 창 한 번만 생성
        try:
            cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name, self.img_size, self.img_size)
            rospy.loginfo(f"[PerLidar] OpenCV window: {self.win_name}")
        except Exception as e:
            rospy.logwarn(f"[PerLidar] OpenCV window create failed: {e}. Disable viz.")
            self.show_rviz = True

    # ---------------- ROS callbacks ----------------
    def CB_lidar_raw(self, msg: LaserScan):
        self.laser_data = msg
        self.processing()

    # ---------------- helpers ----------------
    def pub_lidar_info(self, obstacles):
        json_str = json.dumps(obstacles)
        self.pub_lidar.publish(json_str)

    def calculate_xy_coordinates(self):
        # LaserScan 극좌표 -> (x,y) 변환
        angle = self.laser_data.angle_max  # (드라이버가 max->min 순회라 가정)
        pts = []
        inc = self.laser_data.angle_increment
        for r in self.laser_data.ranges:
            if np.isinf(r) or np.isnan(r) or r <= 0.0:
                angle += inc
                continue
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            pts.append((x, y))
            angle += inc
        return pts

    def divide_ROI(self, points):
        points_np = np.array(points) if len(points) else np.zeros((0,2), dtype=float)
        if len(points_np) == 0:
            empty = np.zeros((0,2), dtype=float)
            return empty, empty, empty, empty, empty

        x = points_np[:, 0]
        y = points_np[:, 1]

        mask_front = (self.length_x_front <= x) & (x <= 0) & (np.abs(y) <= self.length_y_front)
        front_pts = points_np[mask_front]

        mask_front_near = (self.length_x_front_near <= x) & (x <= 0) & (np.abs(y) <= self.length_y_front_near)
        front_pts_near = points_np[mask_front_near]

        mask_left = (self.length_x_left <= x) & (x <= 0) & (self.length_y_left_min <= y) & (y <= self.length_y_left_max)
        left_pts = points_np[mask_left]

        mask_right = (self.length_x_right <= x) & (x <= 0) & (self.length_y_right_min <= y) & (y <= self.length_y_right_max)
        right_pts = points_np[mask_right]

        mask_rotary = (self.length_x_rotary <= x) & (x <= 0) & (self.length_y_rotary_min <= y) & (y <= self.length_y_rotary_max)
        rotary_pts = points_np[mask_rotary]

        return front_pts, left_pts, right_pts, front_pts_near, rotary_pts

    def estimate_obstacle(self, regions):
        # 각 ROI에서 N개 이상이면 True
        results = []
        for region in regions:
            results.append(len(region) >= 4)
        return results

    def estimate_ROI_point_mean(self, pts):
        if pts is None or len(pts) == 0:
            return []
        return np.mean(pts, axis=0).tolist()

    # ---------------- visualization ----------------
    def draw_scene(self, points):
        """points: [(x,y), ...] in meters"""
        if not self.show_rviz:
            return

        img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        cx = cy = self.img_size // 2
        s = self.scale

        # 축 표시(앞=위쪽 느낌으로)
        cv2.line(img, (0, cy), (self.img_size-1, cy), (60,60,60), 1)  # x축
        cv2.line(img, (cx, 0), (cx, self.img_size-1), (60,60,60), 1)  # y축

        # 포인트들
        for x, y in points:
            px = int(cx + y * s)   # y -> 화면 x
            py = int(cy + x * s)   # x -> 화면 y
            if 0 <= px < self.img_size and 0 <= py < self.img_size:
                cv2.circle(img, (px, py), 1, (0, 255, 0), -1)

        # ROI 박스들
        def rect(xmin, ymin, xmax, ymax, color):
            # (로봇좌표 x=앞(+), y=좌(+)) → 화면
            x1 = int(cx + ymin * s)
            y1 = int(cy + xmin * s)
            x2 = int(cx + ymax * s)
            y2 = int(cy + xmax * s)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 1)

        # 전방
        rect(self.length_x_front, -self.length_y_front, 0.0, self.length_y_front, (255,255,0))
        # 좌측
        rect(self.length_x_left,  self.length_y_left_min, 0.0, self.length_y_left_max, (0,255,255))
        # 우측
        rect(self.length_x_right, self.length_y_right_min, 0.0, self.length_y_right_max, (255,0,255))
        # rotary
        rect(self.length_x_rotary, self.length_y_rotary_min, 0.0, self.length_y_rotary_max, (0,0,255))

        # 로봇 원점
        cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)

        # 창에 출력
        cv2.imshow(self.win_name, img)
        # waitKey는 반드시 호출(이벤트 처리)
        # 1ms면 충분. CPU 과점유 시 10~20으로 늘려도 됨
        cv2.waitKey(1)

    # ---------------- main processing ----------------
    def processing(self):
        try:
            points = self.calculate_xy_coordinates()
            front_pts, left_pts, right_pts, front_pts_near, rotary_pts = self.divide_ROI(points)
            obstacles = self.estimate_obstacle((left_pts, front_pts, right_pts, front_pts_near))
            obstacles.append(self.estimate_ROI_point_mean(rotary_pts))
            self.pub_lidar_info(obstacles)

            self.draw_scene(points)  # ← 시각화 호출
        except Exception as e:
            rospy.logdebug(f"[PerLidar] processing error: {e}")

    def on_shutdown(self):
        if self.show_rviz:
            try:
                cv2.destroyWindow(self.win_name)
            except:
                pass
            try:
                cv2.destroyAllWindows()
            except:
                pass

if __name__ == '__main__':
    node = PerLidar()
    rospy.spin()
