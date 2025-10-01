#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneDetectorNode:
    def __init__(self):
        # ---- ROS 설정 ----
        self.image_topic = rospy.get_param("~image_topic", "/limo/color/image_raw")
        self.show_debug  = rospy.get_param("~show_debug", True)
        self.use_birdview = rospy.get_param("~use_birdview", True)  # 원하면 버드뷰 사용
        self.bridge = CvBridge()

        # ---- 서브스크라이버 ----
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)

        # ---- 버드뷰용 ROI (이미지 크기에 맞춰 런타임에 비율로 재계산) ----
        self.src_ratio = np.float32([
            [0.15, 0.70],
            [0.85, 0.70],
            [0.98, 0.98],
            [0.02, 0.98]
        ])  # 좌상, 우상, 우하, 좌하 (비율좌표)
        self.dst_ratio = np.float32([
            [0.10, 0.05],
            [0.90, 0.05],
            [0.90, 0.95],
            [0.10, 0.95]
        ])

        rospy.loginfo("LaneDetectorNode started. Subscribing: %s", self.image_topic)

    # -------------------- 유틸 --------------------
    def get_perspective_M(self, w, h):
        src = (self.src_ratio * [w, h]).astype(np.float32)
        dst = (self.dst_ratio * [w, h]).astype(np.float32)
        M  = cv2.getPerspectiveTransform(src, dst)
        iM = cv2.getPerspectiveTransform(dst, src)
        return M, iM

    def make_roi_mask(self, img):
        h, w = img.shape[:2]
        # 트래피조이드 ROI (차선 있는 하부만)
        roi_poly = np.array([[
            (int(0.10*w), int(0.65*h)),
            (int(0.90*w), int(0.65*h)),
            (int(0.98*w), int(0.98*h)),
            (int(0.02*w), int(0.98*h))
        ]], dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, roi_poly, 255)
        return mask

    def color_threshold(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 흰색(밝음 + 채도 낮음)
        white_lower = np.array([0,   0, 190], dtype=np.uint8)
        white_upper = np.array([180, 40, 255], dtype=np.uint8)
        mask_white = cv2.inRange(hsv, white_lower, white_upper)

        # 노란색 (원한다면 같이 사용; 검정바닥+흰선이면 없어도 충분)
        yellow_lower = np.array([15, 80, 120], dtype=np.uint8)
        yellow_upper = np.array([35, 255, 255], dtype=np.uint8)
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

        mask = cv2.bitwise_or(mask_white, mask_yellow)

        # 노이즈 제거
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def detect_lines(self, edge):
        # 허프 확률적 직선
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, threshold=40,
                                minLineLength=40, maxLineGap=20)
        return lines

    def draw_lines(self, canvas, lines, color=(0,255,0), thickness=3):
        if lines is None:
            return
        for l in lines:
            x1,y1,x2,y2 = l[0]
            cv2.line(canvas, (x1,y1), (x2,y2), color, thickness, cv2.LINE_AA)

    # -------------------- 콜백 --------------------
    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        h, w = frame.shape[:2]
        vis  = frame.copy()

        # (선택) 버드뷰
        if self.use_birdview:
            M, iM = self.get_perspective_M(w, h)
            warped = cv2.warpPerspective(frame, M, (w, h), flags=cv2.INTER_LINEAR)
            work = warped
        else:
            work = frame

        # 색상 기반 마스크
        mask_color = self.color_threshold(work)

        # ROI 적용
        roi_mask = self.make_roi_mask(mask_color)
        masked = cv2.bitwise_and(mask_color, roi_mask)

        # 엣지
        edge = cv2.Canny(masked, 80, 160, L2gradient=True)

        # 선 검출
        lines = self.detect_lines(edge)

        # 시각화: 원근 변환 여부에 따라 다른 캔버스에 그림
        draw_canvas = work.copy()
        self.draw_lines(draw_canvas, lines, (0,255,0), 3)

        # (선택) 버드뷰를 원래 시점으로 되돌려 오버레이
        if self.use_birdview:
            unwarp_draw = cv2.warpPerspective(draw_canvas, iM, (w, h), flags=cv2.INTER_LINEAR)
            overlay = frame.copy()
            alpha = 0.6
            vis = cv2.addWeighted(overlay, 1.0, unwarp_draw, alpha, 0)
        else:
            overlay = frame.copy()
            alpha = 0.6
            vis = cv2.addWeighted(overlay, 1.0, draw_canvas, alpha, 0)

        # 디버그 창들
        if self.show_debug:
            cv2.imshow("frame", frame)
            if self.use_birdview:
                cv2.imshow("warped", work)
            cv2.imshow("mask_color", mask_color)
            cv2.imshow("masked_roi", masked)
            cv2.imshow("edge", edge)
            cv2.imshow("lanes_viz", vis)

            # X11 원격이면 키보드 입력이 안 잡힐 수도 있음. 그래도 waitKey(1)는 필수.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested quit")

    def cleanup(self):
        cv2.destroyAllWindows()

def main():
    rospy.init_node("lane_detector_node", anonymous=False)
    node = LaneDetectorNode()
    rospy.on_shutdown(node.cleanup)
    rospy.loginfo("lane_detector_node running.")
    rospy.spin()

if __name__ == "__main__":
    main()
