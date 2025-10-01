#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class CameraViewer:
    def __init__(self):
        # 파라미터
        self.image_topic = rospy.get_param("~image_topic", "/limo/color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/limo/color/camera_info")
        self.use_cam_info = rospy.get_param("~use_cam_info", True)
        self.window_name = rospy.get_param("~window_name", "Camera Viewer")
        self.resize_w = rospy.get_param("~resize_width", 0)   # 0이면 리사이즈 안함
        self.resize_h = rospy.get_param("~resize_height", 0)

        self.bridge = CvBridge()

        # 카메라 내부파라미터
        self.K = None  # 3x3
        self.D = None  # distortion
        self.R = None  # 3x3
        self.P = None  # 3x4
        self.cinfo_size = None  # (w,h)
        self.map1 = None
        self.map2 = None
        self.map_size = None  # (w,h)

        # FPS 계산용
        self.last_t = None
        self.fps = 0.0

        # 구독
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        if self.use_cam_info and self.camera_info_topic:
            self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.info_cb, queue_size=1)

        rospy.loginfo("camera_view.py started.")
        rospy.loginfo("  image_topic: %s", self.image_topic)
        if self.use_cam_info:
            rospy.loginfo("  camera_info_topic: %s", self.camera_info_topic)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def info_cb(self, msg: CameraInfo):
        # CameraInfo 갱신
        self.K = np.array(msg.K, dtype=np.float32).reshape(3, 3)
        self.D = np.array(msg.D, dtype=np.float32).reshape(-1, 1)
        self.R = np.array(msg.R, dtype=np.float32).reshape(3, 3)
        self.P = np.array(msg.P, dtype=np.float32).reshape(3, 4)
        self.cinfo_size = (msg.width, msg.height)

        # 이미지를 한 번 받아야 map을 만들 수 있음(사이즈 필요)
        rospy.loginfo_throttle(5.0, "CameraInfo received. Waiting for first image to build undistort map...")

    def _maybe_build_undistort_map(self, w, h):
        # 언디스토션 맵이 없거나 사이즈가 달라지면 재생성
        if not self.use_cam_info or self.K is None or self.D is None:
            return
        if self.map1 is not None and self.map_size == (w, h):
            return

        if self.cinfo_size is not None and self.cinfo_size != (w, h):
            rospy.logwarn_throttle(5.0, f"CameraInfo size {self.cinfo_size} != image size {(w,h)}. Skip undistort.")
            return

        try:
            newK = self.K.copy()  # 간단하게 동일 내재행렬 사용(필요하면 getOptimalNewCameraMatrix로 조정)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K, self.D, None, newK, (w, h), cv2.CV_16SC2
            )
            self.map_size = (w, h)
            rospy.loginfo("Undistort map built for size: %s", self.map_size)
        except Exception as e:
            rospy.logwarn("Failed to build undistort map: %s", e)
            self.map1, self.map2, self.map_size = None, None, None

    def _put_hud(self, img, stamp, extra=""):
        # FPS 텍스트 & 타임스탬프
        h, w = img.shape[:2]
        t = time.time()
        if self.last_t is not None:
            dt = t - self.last_t
            if dt > 1e-6:
                # 지터 줄이려고 지수평활
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if self.fps > 0 else 1.0 / dt
        self.last_t = t

        txt1 = f"FPS: {self.fps:5.2f}"
        txt2 = f"stamp: {stamp.secs}.{stamp.nsecs:09d}"
        txt3 = f"topic: {self.image_topic}"
        if extra:
            txt3 += f" | {extra}"

        cv2.putText(img, txt1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
        cv2.putText(img, txt2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(img, txt3, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1, cv2.LINE_AA)

    def image_cb(self, msg: Image):
        # 인코딩에 따라 분기
        enc = msg.encoding.lower() if msg.encoding else "bgr8"

        try:
            if "16uc" in enc:  # 16UC1(depth mm)
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                # 시각화용 정규화 + 컬러맵
                # 유효한 깊이만 대상으로 정규화(0은 invalid로 가정)
                depth_mask = depth > 0
                if np.any(depth_mask):
                    d_min = float(np.min(depth[depth_mask]))
                    d_max = float(np.max(depth[depth_mask]))
                else:
                    d_min, d_max = 0.0, 1.0
                depth_norm = np.zeros_like(depth, dtype=np.uint8)
                if d_max > d_min:
                    depth_norm = cv2.convertScaleAbs(depth, alpha=255.0/(d_max-d_min), beta=-255.0*d_min/(d_max-d_min))
                depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                disp = depth_color
                extra = f"Depth(16UC1) range:[{d_min:.1f},{d_max:.1f}]"
            elif "32fc" in enc:  # 32FC1(depth meter)
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth_mask = np.isfinite(depth) & (depth > 0)
                if np.any(depth_mask):
                    d_min = float(np.nanmin(depth[depth_mask]))
                    d_max = float(np.nanmax(depth[depth_mask]))
                else:
                    d_min, d_max = 0.0, 1.0
                depth_vis = np.zeros_like(depth, dtype=np.float32)
                if d_max > d_min:
                    depth_vis = (depth - d_min) / (d_max - d_min)
                depth_vis = np.clip(depth_vis, 0.0, 1.0)
                depth_u8 = (depth_vis * 255.0).astype(np.uint8)
                depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
                disp = depth_color
                extra = f"Depth(32FC1) range:[{d_min:.2f},{d_max:.2f}]m"
            else:
                # 일반 컬러
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # 언디스토션 맵 준비(최초 1회)
                self._maybe_build_undistort_map(img.shape[1], img.shape[0])
                if self.map1 is not None and self.map2 is not None:
                    img = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

                disp = img
                extra = "BGR8"

            # 리사이즈 옵션
            if self.resize_w > 0 and self.resize_h > 0:
                disp = cv2.resize(disp, (int(self.resize_w), int(self.resize_h)), interpolation=cv2.INTER_AREA)

            # HUD
            self._put_hud(disp, msg.header.stamp, extra)

            cv2.imshow(self.window_name, disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):  # ESC or q
                rospy.signal_shutdown("user exit")
            elif key == ord('s'):
                path = "/tmp/frame_{:.0f}.png".format(time.time()*1000)
                cv2.imwrite(path, disp)
                rospy.loginfo("Saved: %s", path)

        except Exception as e:
            rospy.logwarn_throttle(2.0, "image_cb error: %s (encoding=%s)", e, enc)

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("camera_viewer", anonymous=True)
    viewer = CameraViewer()
    viewer.spin()
