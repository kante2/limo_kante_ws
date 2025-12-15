#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
from math import *


class LimoLanePerception:
    def __init__(self):
        # 여기서는 init_node 안 함 (메인에서 호출)
        # rospy.init_node("limo_lane_perception_node")

        # === ROS IO ===
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_cb)
        rospy.on_shutdown(self.shutdown_hook)

        # perception 결과 퍼블리시
        self.center_error_pub = rospy.Publisher("/lane/center_error", Float32, queue_size=1)
        self.valid_pub = rospy.Publisher("/lane/valid", Bool, queue_size=1)

        # === ROS 변수 ===
        self.bridge = CvBridge()

        # === 상태 플래그 ===
        self.camera_flag = False
        self.camera_msg = None

        # =================================================================
        # [튜닝 파라미터] (원본 그대로)
        # =================================================================
        self.img_w = 640
        self.img_h = 480
        self.center_block_half_width_px = 140

        # === Sliding-window ===
        self.num_windows = 12
        self.window_margin = 80
        self.minpix_recenter = 50
        self.min_lane_sep = 60
        self.center_ema_alpha = 0.8
        self.center_ema = None  # (y, x) EMA 저장용

        # === ROI polygon (ratios) ===
        self.roi_top_y_ratio     = 0.7
        self.roi_left_top_ratio  = 0.2
        self.roi_right_top_ratio = 0.8
        self.roi_left_bot_ratio  = -0.50
        self.roi_right_bot_ratio = 1.50

        self.lane_width_px = 540.0

        # === Color thresholds (HSV) ===
        self.yellow_lower = np.array([18,  60, 140], dtype=np.uint8)
        self.yellow_upper = np.array([40, 255, 255], dtype=np.uint8)

        self.white_lower  = np.array([0,   0, 160], dtype=np.uint8)
        self.white_upper  = np.array([179, 100, 255], dtype=np.uint8)

        self.black_track_lower = np.array([0,   0,   0], dtype=np.uint8)
        self.black_track_upper = np.array([179, 255, 85], dtype=np.uint8)

        rospy.loginfo("Limo Lane Perception 초기화 완료 (차선 인식만 수행).")

    # ---------------- ROS Callbacks ----------------
    def camera_cb(self, msg):
        self.camera_msg = msg
        self.camera_flag = True

    def shutdown_hook(self):
        rospy.loginfo("lane_perception 종료, OpenCV 윈도우 닫기.")
        cv2.destroyAllWindows()

    # ---------------- Camera Perception ----------------
    def process_camera_features(self, bgr_img):
        """
        return: (center_error_px, valid)
        center_error_px: float (이미지 중앙 대비 차선 중앙의 픽셀 오프셋, +면 우측)
        valid: bool  (차선 검출 성공 여부)
        """
        h, w = bgr_img.shape[:2]
        self.img_w, self.img_h = w, h

        # 0) 카메라 프레임에서 중앙 ±구간의 흰색 제거
        hsv_cam = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        cx = w // 2
        half = int(self.center_block_half_width_px)
        x1 = max(0, cx - half)
        x2 = min(w, cx + half)

        center_roi = hsv_cam[:, x1:x2]
        center_white_mask = cv2.inRange(center_roi, self.white_lower, self.white_upper)
        center_roi[center_white_mask > 0] = (0, 0, 0)
        hsv_cam[:, x1:x2] = center_roi

        bgr_img = cv2.cvtColor(hsv_cam, cv2.COLOR_HSV2BGR)

        # 1) ROI 설정 및 BEV 변환
        roi_poly = self.make_roi_polygon(h, w)
        bev_bgr = self.warp_to_bev(bgr_img, roi_poly)
        if bev_bgr is None:
            rospy.logwarn("BEV 변환 실패 - 조향 0.0 사용")
            return 0.0, False

        # 2) 차선 바이너리
        bev_binary = self.binarize_lanes(bev_bgr)

        # 3) 슬라이딩 윈도우 + 디버그
        debug_bev_img, left_centers, right_centers = \
            self.run_sliding_window_and_visualize(bev_binary)

        # 4) 차선 중앙점 계산
        center_point = self.compute_center_point(left_centers, right_centers, bev_binary.shape[0])

        # === HSV 값 확인용 클릭 ===
        hsv_img_for_click = hsv_cam.copy()

        def print_hsv_value(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                hsv_pixel = hsv_img_for_click[y, x]
                rospy.loginfo(
                    f"=== Clicked Pixel HSV === H: {hsv_pixel[0]}, S: {hsv_pixel[1]}, V: {hsv_pixel[2]}"
                )

        cv2.polylines(bgr_img, [roi_poly], True, (0, 255, 0), 2)
        cv2.imshow("HSV_Finder & ROI_Tuner (Click on Lane)", bgr_img)
        cv2.setMouseCallback("HSV_Finder & ROI_Tuner (Click on Lane)", print_hsv_value)

        cv2.imshow("Binary (BEV) - HSV Tuning", bev_binary)

        # 5) 곡률 폴리곤 (디버그)
        left_fit, _ = self.compute_curvature_from_centers(left_centers, bev_binary.shape[0])
        right_fit, _ = self.compute_curvature_from_centers(right_centers, bev_binary.shape[0])

        if left_fit is not None:
            self.draw_polynomial(debug_bev_img, left_fit, (0, 0, 255))
        if right_fit is not None:
            self.draw_polynomial(debug_bev_img, right_fit, (0, 255, 0))

        # 6) 스티어 계산
        valid = False
        center_error = 0.0
        if center_point is not None:
            cy, cx_center = center_point
            img_cx = bev_binary.shape[1] * 0.5
            center_error = float(cx_center) - float(img_cx)
            valid = True

            cv2.drawMarker(
                debug_bev_img,
                (int(img_cx), int(bev_binary.shape[0] // 2)),
                (255, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=2,
            )
            cv2.drawMarker(
                debug_bev_img,
                (int(cx_center), int(cy)),
                (255, 0, 255),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=12,
                thickness=2,
            )
        else:
            center_error = 0.0

        cv2.imshow("Lane Detection (Sliding Windows)", debug_bev_img)

        return center_error, valid

    # ---------------- Helper: ROI / BEV ----------------
    def make_roi_polygon(self, h, w):
        y_top = int(h * self.roi_top_y_ratio)
        y_bot = h - 1
        x_lt  = int(w * self.roi_left_top_ratio)
        x_rt  = int(w * self.roi_right_top_ratio)
        x_lb  = int(w * self.roi_left_bot_ratio)
        x_rb  = int(w * self.roi_right_bot_ratio)
        return np.array([[x_lb, y_bot],
                         [x_lt, y_top],
                         [x_rt, y_top],
                         [x_rb, y_bot]], np.int32)

    def warp_to_bev(self, bgr, roi_poly):
        if bgr is None:
            return None

        h, w = bgr.shape[:2]
        BL, TL, TR, BR = roi_poly.astype(np.float32)
        src = np.float32([TL, TR, BR, BL])
        dst = np.float32([
            [0,       0      ],
            [w - 1.0, 0      ],
            [w - 1.0, h - 1.0],
            [0,       h - 1.0]
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(bgr, M, (w, h))
        return bev

    # ---------------- Helper: Binarization ----------------
    def binarize_lanes(self, bev_bgr):
        if bev_bgr is None:
            rospy.logwarn("Binarize: BEV 이미지가 비어있습니다.")
            return np.zeros((self.img_h, self.img_w), dtype=np.uint8)

        h, w = bev_bgr.shape[:2]
        hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        white_mask  = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        line_mask = cv2.bitwise_or(yellow_mask, white_mask)

        track_mask = cv2.inRange(hsv, self.black_track_lower, self.black_track_upper)

        kernel_size = 21
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        track_area_mask = cv2.morphologyEx(track_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        track_area_mask = cv2.morphologyEx(track_area_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        cv2.imshow("Track Area Mask (Filtering)", track_area_mask)

        final_binary = cv2.bitwise_and(line_mask, line_mask, mask=track_area_mask)

        cx = w // 2
        half = int(self.center_block_half_width_px)
        x1 = max(0, cx - half)
        x2 = min(w, cx + half)
        final_binary[:, x1:x2] = 0

        kernel_small = np.ones((3, 3), np.uint8)
        final_binary = cv2.morphologyEx(final_binary, cv2.MORPH_OPEN, kernel_small, iterations=1)

        return final_binary

    # ---------------- Helper: Sliding Window ----------------
    def run_sliding_window_and_visualize(self, binary_img):
        out_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        histogram = np.sum(binary_img[binary_img.shape[0] // 2:, :], axis=0)

        midpoint = int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        n_windows = self.num_windows
        window_height = int(binary_img.shape[0] / n_windows)

        nonzero = binary_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        margin = self.window_margin
        minpix = self.minpix_recenter

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        left_centers = []
        right_centers = []

        for window in range(n_windows):
            win_y_low = binary_img.shape[0] - (window + 1) * window_height
            win_y_high = binary_img.shape[0] - window * window_height

            win_xleft_low  = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low  = rightx_current - margin
            win_xright_high = rightx_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low),
                          (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low),
                          (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                centery = int(np.mean(nonzeroy[good_left_inds]))
                left_centers.append((centery, leftx_current))

            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                centery = int(np.mean(nonzeroy[good_right_inds]))
                right_centers.append((centery, rightx_current))

        left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) > 0 else []
        right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) > 0 else []

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        return out_img, left_centers, right_centers

    # ---------------- Helper: Center Point ----------------
    def compute_center_point(self, left_centers, right_centers, h):
        cy, cx = None, None

        if left_centers and right_centers:
            ly, lx = left_centers[-1]
            ry, rx = right_centers[-1]
            cy = (ly + ry) * 0.5
            cx = (lx + rx) * 0.5
        elif left_centers and not right_centers:
            ly, lx = left_centers[-1]
            cy = ly
            cx = lx + self.lane_width_px * 0.5
        elif right_centers and not left_centers:
            ry, rx = right_centers[-1]
            cy = ry
            cx = rx - self.lane_width_px * 0.5
        else:
            return None

        if self.center_ema is None:
            self.center_ema = (cy, cx)
        else:
            prev_y, prev_x = self.center_ema
            alpha = self.center_ema_alpha
            new_y = alpha * prev_y + (1.0 - alpha) * cy
            new_x = alpha * prev_x + (1.0 - alpha) * cx
            self.center_ema = (new_y, new_x)

        return self.center_ema

    # ---------------- Helper: Curvature & Drawing ----------------
    def compute_curvature_from_centers(self, centers, h):
        if centers is None or len(centers) < 3:
            return None, None

        ys = np.array([c[0] for c in centers], dtype=np.float32)
        xs = np.array([c[1] for c in centers], dtype=np.float32)

        fit = np.polyfit(ys, xs, 2)

        y_eval = float(h - 1)
        A, B, _ = fit
        denom = (2 * A * y_eval + B) ** 2
        curvature = np.inf
        if denom > 1e-6:
            curvature = ((1 + denom) ** 1.5) / abs(2 * A)

        return fit, curvature

    def draw_polynomial(self, img, fit, color):
        if fit is None:
            return

        h, w = img.shape[:2]
        plot_y = np.linspace(0, h - 1, h).astype(np.float32)
        A, B, C = fit
        plot_x = A * plot_y ** 2 + B * plot_y + C

        pts = []
        for y, x in zip(plot_y, plot_x):
            xi = int(np.clip(x, 0, w - 1))
            yi = int(np.clip(y, 0, h - 1))
            pts.append((xi, yi))

        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], color, 2)

    # ---------------- lane_perception_step ----------------
    def lane_perception_step(self):
        """
        한 번 호출할 때마다:
        - 최신 카메라 프레임 처리
        - center_error / valid 계산
        - /lane/center_error, /lane/valid 퍼블리시
        - OpenCV 윈도우 갱신
        """
        if self.camera_flag and self.camera_msg is not None:
            try:
                bgr_img = self.bridge.compressed_imgmsg_to_cv2(
                    self.camera_msg, desired_encoding='bgr8'
                )
                center_error, valid = self.process_camera_features(bgr_img)
            except Exception as e:
                rospy.logwarn(f"카메라 처리 에러: {e}")
                center_error, valid = 0.0, False
        else:
            center_error, valid = 0.0, False

        self.center_error_pub.publish(Float32(data=float(center_error)))
        self.valid_pub.publish(Bool(data=bool(valid)))

        # ESC로 전체 노드 종료
        if cv2.waitKey(1) & 0xFF == 27:
            rospy.signal_shutdown("ESC pressed in perception")


def main():
    rospy.init_node("limo_lane_perception")
    node = LimoLanePerception()
    rate_hz = rospy.get_param("~loop_rate", 30.0)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        node.lane_perception_step()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
