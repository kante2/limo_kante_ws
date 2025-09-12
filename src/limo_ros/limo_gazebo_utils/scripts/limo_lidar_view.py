#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import LaserScan

class LidarViewer:
    def __init__(self):
        # --- Params ---
        self.scan_topic = rospy.get_param("~scan_topic", "/limo/scan")
        self.window_name = rospy.get_param("~window_name", "LIMO LiDAR")
        self.view_radius_m = float(rospy.get_param("~view_radius_m", 10.0))   # 화면에 보여줄 반경(m)
        self.img_size = int(rospy.get_param("~img_size", 800))               # 정사각 출력
        self.draw_grid = bool(rospy.get_param("~draw_grid", True))
        self.grid_step_m = float(rospy.get_param("~grid_step_m", 1.0))
        self.point_size_px = int(rospy.get_param("~point_size_px", 2))
        self.connect_lines = bool(rospy.get_param("~connect_lines", False))  # 인접빔 라인 연결 표시
        self.publish_filtered = bool(rospy.get_param("~publish_filtered", False))
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.05))      # 필터용
        self.max_range_m = float(rospy.get_param("~max_range_m", 0.0))       # 0이면 scan.range_max 사용

        # --- State ---
        self.last_scan = None
        self.fps = 0.0
        self.last_t = None

        # --- ROS ---
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)
        self.pub_filtered = None
        if self.publish_filtered:
            self.pub_filtered = rospy.Publisher(self.scan_topic + "_filtered", LaserScan, queue_size=1)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        rospy.loginfo("limo_lidar_view: subscribe %s", self.scan_topic)

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg
        if self.publish_filtered:
            filt = self.filter_scan(msg)
            self.pub_filtered.publish(filt)

        # Draw immediately for responsiveness
        self.draw_scan(msg)

    def filter_scan(self, scan: LaserScan) -> LaserScan:
        """간단 필터: NaN/Inf 제거, min/max 범위 밖 값 클램프"""
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        ranges = np.array(scan.ranges, dtype=np.float32)

        # 유효성
        valid = np.isfinite(ranges)
        ranges[~valid] = 0.0

        # 절대 최소/최대
        rmin = max(self.min_range_m, scan.range_min)
        rmax = (self.max_range_m if self.max_range_m > 0.0 else scan.range_max)
        ranges[ranges < rmin] = 0.0
        ranges[ranges > rmax] = 0.0

        out.ranges = ranges.tolist()
        # intensities 길이 맞으면 그대로 복사
        if len(scan.intensities) == len(scan.ranges):
            out.intensities = list(scan.intensities)
        return out

    def draw_scan(self, scan: LaserScan):
        # FPS update
        tnow = time.time()
        if self.last_t is not None:
            dt = tnow - self.last_t
            if dt > 1e-6:
                self.fps = 0.9*self.fps + 0.1*(1.0/dt) if self.fps > 0 else (1.0/dt)
        self.last_t = tnow

        # Image canvas
        img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        cx, cy = self.img_size // 2, self.img_size // 2
        # 스케일: m -> px
        scale = (self.img_size * 0.45) / max(0.1, self.view_radius_m)

        # Grid
        if self.draw_grid:
            steps = int(self.view_radius_m // self.grid_step_m)
            for i in range(1, steps+1):
                r = int(i * self.grid_step_m * scale)
                cv2.circle(img, (cx, cy), r, (50, 50, 50), 1, lineType=cv2.LINE_AA)
            # 축(x: 전방, y: 좌측)
            cv2.line(img, (cx, cy), (cx, cy - int(self.view_radius_m*scale)), (60, 60, 60), 1)
            cv2.line(img, (cx, cy), (cx + int(self.view_radius_m*scale), cy), (60, 60, 60), 1)

        # Convert ranges -> points
        n = len(scan.ranges)
        angles = scan.angle_min + scan.angle_increment * np.arange(n, dtype=np.float32)
        ranges = np.array(scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges > 0.0)

        # 시야 반경 제한(뷰 밖은 그리지 않음)
        rmax_view = max(self.view_radius_m, 0.1)
        ranges_plot = np.clip(ranges, 0.0, rmax_view)
        valid = valid & (ranges_plot > 0.0)

        x = ranges_plot * np.cos(angles)   # 전방 +x
        y = ranges_plot * np.sin(angles)   # 좌측 +y
        # img 좌표로 변환 (y축 반전)
        px = (cx + (x * scale)).astype(np.int32)
        py = (cy - (y * scale)).astype(np.int32)

        # intensities가 있다면 컬러맵
        color_pts = (255, 255, 255)
        if len(scan.intensities) == n and np.any(np.isfinite(scan.intensities)):
            intens = np.array(scan.intensities, dtype=np.float32)
            intens[np.isnan(intens)] = 0.0
            if np.max(intens) > np.min(intens):
                norm = (intens - np.min(intens)) / (np.max(intens) - np.min(intens))
            else:
                norm = np.zeros_like(intens)
            cmap = (cv2.applyColorMap((norm*255).astype(np.uint8), cv2.COLORMAP_TURBO)).reshape(-1,3)
        else:
            cmap = None

        # Draw points / lines
        last_p = None
        count = 0
        for i in range(n):
            if not valid[i]:
                last_p = None
                continue
            p = (int(px[i]), int(py[i]))
            if 0 <= p[0] < self.img_size and 0 <= p[1] < self.img_size:
                if cmap is not None:
                    c = tuple(int(v) for v in cmap[i])
                else:
                    c = color_pts
                cv2.circle(img, p, self.point_size_px, c, -1, lineType=cv2.LINE_AA)
                if self.connect_lines and last_p is not None:
                    cv2.line(img, last_p, p, (120, 120, 120), 1, lineType=cv2.LINE_AA)
                last_p = p
                count += 1
            else:
                last_p = None

        # HUD
        txt1 = f"Topic: {self.scan_topic}"
        txt2 = f"FPS: {self.fps:5.2f}   Points: {count}/{n}"
        txt3 = f"View radius: {self.view_radius_m:.1f} m"
        cv2.putText(img, txt1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150,150,255), 1, cv2.LINE_AA)
        cv2.putText(img, txt2, (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150,255,150), 1, cv2.LINE_AA)
        cv2.putText(img, txt3, (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 4, (0, 255, 0), -1, lineType=cv2.LINE_AA)  # 로봇 위치 표시

        cv2.imshow(self.window_name, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC/q
            rospy.signal_shutdown("user exit")
        elif key == ord('s'):
            path = "/tmp/lidar_{:.0f}.png".format(time.time()*1000)
            cv2.imwrite(path, img)
            rospy.loginfo("Saved: %s", path)

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("limo_lidar_view", anonymous=True)
    node = LidarViewer()
    node.spin()
