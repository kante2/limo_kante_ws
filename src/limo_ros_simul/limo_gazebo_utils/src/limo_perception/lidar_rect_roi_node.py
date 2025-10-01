#!/usr/bin/env python3
# lidar_rect_roi_node.py
import math
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, Int32, String
from cv_bridge import CvBridge

class LidarRectROI:
    def __init__(self):
        # ====== 기본 입력 ======
        self.scan_topic = rospy.get_param("~scan_topic", "/limo/scan")

        # ====== 사각형 ROI (x_min, y_min, x_max, y_max) [m] ======
        # 좌표계: x=전방(+), y=좌측(+)/우측(-)
        self.left_roi  = self._load_rect("~left_roi",  [0.3,  0.20, 2.5,  0.80])
        self.front_roi = self._load_rect("~front_roi", [0.3, -0.20, 2.5,  0.20])
        self.right_roi = self._load_rect("~right_roi", [0.3, -0.80, 2.5, -0.20])

        # front ROI 개수로 slow/stop 판단
        self.min_points_slow = int(rospy.get_param("~min_points_slow", 10))
        self.min_points_stop = int(rospy.get_param("~min_points_stop", 15))

        # ====== 시각화 설정 ======
        # 그릴 공간(m): x 0~x_max, y -y_max~+y_max
        self.viz_x_max = float(rospy.get_param("~viz_x_max", 5.0))
        self.viz_y_max = float(rospy.get_param("~viz_y_max", 3.0))
        # 픽셀 해상도(px)
        self.viz_w     = int(rospy.get_param("~viz_width", 800))
        self.viz_h     = int(rospy.get_param("~viz_height", 480))
        # 표시할 최대 포인트 거리(시각화 한정)
        self.viz_r_max = float(rospy.get_param("~viz_r_max", 10.0))
        # OpenCV 창 표시 여부(헤드리스면 False 권장)
        self.use_imshow = bool(rospy.get_param("~use_imshow", False))
        # Image 토픽 퍼블리시 여부
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.debug_image_topic   = rospy.get_param("~debug_image_topic", "/lidar_rect_roi/debug_image")

        # 색상(BGR)
        self.col_left  = (40, 180, 255)   # 주황
        self.col_front = (50, 220, 50)    # 초록
        self.col_right = (255, 80, 80)    # 파랑계
        self.col_all   = (180, 180, 180)  # 배경 포인트(ROI 밖)
        self.col_axes  = (200, 200, 200)  # 축/그리드
        self.col_text  = (255, 255, 255)

        # 퍼블리셔
        self.pub_left_cnt   = rospy.Publisher("/전방왼쪽_lidar_left_roi/count",    Int32,  queue_size=1)
        self.pub_front_cnt  = rospy.Publisher("/전방lidar_front_roi/count",       Int32,  queue_size=1)
        self.pub_right_cnt  = rospy.Publisher("/전방오른쪽lidar_right_roi/count",  Int32,  queue_size=1)
        self.pub_slow_flag  = rospy.Publisher("/lidar_slow_flag",   Bool,   queue_size=1)
        self.pub_stop_flag  = rospy.Publisher("/lidar_stop_flag",   Bool,   queue_size=1)
        self.pub_obs_loc    = rospy.Publisher("/lidar_obs_location", String, queue_size=1)

        # 디버그 이미지 퍼블리셔
        self.bridge = CvBridge()
        self.pub_img = rospy.Publisher(self.debug_image_topic, Image, queue_size=1) if self.publish_debug_image else None

        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)
        rospy.loginfo("lidar_rect_roi started. left=%s front=%s right=%s",
                      self.left_roi, self.front_roi, self.right_roi)

        # 미리 스케일 계산
        self._compute_scale()

    # ---------- 유틸 ----------
    def _load_rect(self, key, default_rect):
        rect = rospy.get_param(key, default_rect)
        if not (isinstance(rect, (list, tuple)) and len(rect) == 4):
            rospy.logwarn("%s must be [x_min, y_min, x_max, y_max], using default %s", key, default_rect)
            rect = default_rect
        x0, y0, x1, y1 = [float(v) for v in rect]
        x_min, x_max = (x0, x1) if x0 <= x1 else (x1, x0)
        y_min, y_max = (y0, y1) if y0 <= y1 else (y1, y0)
        return [x_min, y_min, x_max, y_max]

    def _compute_scale(self):
        # meter -> pixel 변환; y는 상단이 +가 되도록 뒤집을 것(화면 좌표계 보정)
        self.scale_x = (self.viz_w - 1) / max(1e-6, self.viz_x_max)            # px per meter
        self.scale_y = (self.viz_h - 1) / max(1e-6, self.viz_y_max * 2.0)      # y: [-y_max, y_max] 폭

    def _world2img(self, x, y):
        """로봇 프레임(m) → 이미지 픽셀(u,v).
        x∈[0,x_max], y∈[-y_max,+y_max]만 표시."""
        u = int(np.clip(x * self.scale_x, 0, self.viz_w - 1))
        # y: -y_max → 하단, +y_max → 상단 (영상 좌표는 아래로 증가하므로 뒤집기)
        v = int(np.clip((self.viz_y_max - y) * self.scale_y, 0, self.viz_h - 1))
        return u, v

    @staticmethod
    def _in_rect(x, y, rect):
        x_min, y_min, x_max, y_max = rect
        return (x_min <= x <= x_max) and (y_min <= y <= y_max)

    def _draw_axes(self, img):
        # 0m, x/y 눈금(1m 간격) 그리기
        # x축: y=0, y축: x=0
        for xm in range(0, int(self.viz_x_max) + 1):
            u0, v0 = self._world2img(xm, -self.viz_y_max)
            u1, v1 = self._world2img(xm, +self.viz_y_max)
            cv2.line(img, (u0, v0), (u1, v1), self.col_axes, 1, cv2.LINE_AA)
            cv2.putText(img, f"x{xm}", (u0+2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.col_axes, 1, cv2.LINE_AA)
        for yi in range(-int(self.viz_y_max), int(self.viz_y_max)+1):
            u0, v0 = self._world2img(0.0, yi)
            u1, v1 = self._world2img(self.viz_x_max, yi)
            cv2.line(img, (u0, v0), (u1, v1), self.col_axes, 1, cv2.LINE_AA)
            cv2.putText(img, f"y{yi}", (2, v0-2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.col_axes, 1, cv2.LINE_AA)

    def _draw_rect(self, img, rect, color, label):
        (x0, y0, x1, y1) = rect
        p0 = self._world2img(x0, y0)
        p1 = self._world2img(x1, y1)
        cv2.rectangle(img, p0, p1, color, 2, cv2.LINE_AA)
        # 라벨
        lx, ly = self._world2img(x0, y1)  # 좌상단 근처
        cv2.putText(img, label, (lx+4, ly+14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # ---------- 콜백 ----------
    def cb_scan(self, scan: LaserScan):
        left_cnt = 0
        front_cnt = 0
        right_cnt = 0

        # 캔버스 생성 (BGR)
        canvas = np.zeros((self.viz_h, self.viz_w, 3), dtype=np.uint8)
        self._draw_axes(canvas)
        # ROI 사각형
        self._draw_rect(canvas, self.left_roi,  self.col_left,  "LEFT")
        self._draw_rect(canvas, self.front_roi, self.col_front, "FRONT")
        self._draw_rect(canvas, self.right_roi, self.col_right, "RIGHT")

        a = scan.angle_min
        for r in scan.ranges:
            if r is None or r <= 0.0 or math.isinf(r) or math.isnan(r):
                a += scan.angle_increment
                continue

            # 극 → 직교
            x = r * math.cos(a)
            y = r * math.sin(a)

            # 시각화 범위 내부만 그림
            if not (0.0 <= x <= self.viz_x_max and -self.viz_y_max <= y <= self.viz_y_max):
                a += scan.angle_increment
                continue

            # 기본 점(회색)
            u, v = self._world2img(x, y)
            color = self.col_all

            # ROI 포함 여부 체크 + 카운트
            in_left  = self._in_rect(x, y, self.left_roi)
            in_front = self._in_rect(x, y, self.front_roi)
            in_right = self._in_rect(x, y, self.right_roi)

            if in_left:  left_cnt  += 1
            if in_front: front_cnt += 1
            if in_right: right_cnt += 1

            # ROI별 색상 우선순위(겹칠 수 있으니 front > left > right 등으로 정하고 싶으면 바꿔도 됨)
            if in_front:      color = self.col_front
            elif in_left:     color = self.col_left
            elif in_right:    color = self.col_right

            cv2.circle(canvas, (u, v), 2, color, -1, cv2.LINE_AA)

            a += scan.angle_increment

        # 텍스트 요약
        cv2.putText(canvas, f"LEFT: {left_cnt}",   (10, 20),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.col_left,  2, cv2.LINE_AA)
        cv2.putText(canvas, f"FRONT: {front_cnt}", (10, 45),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.col_front, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"RIGHT: {right_cnt}", (10, 70),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.col_right, 2, cv2.LINE_AA)

        # 카운트 퍼블리시
        self.pub_left_cnt.publish(Int32(left_cnt))
        self.pub_front_cnt.publish(Int32(front_cnt))
        self.pub_right_cnt.publish(Int32(right_cnt))

        # 플래그 퍼블리시(front 기반)
        slow_flag = (front_cnt >= self.min_points_slow)
        stop_flag = (front_cnt >= self.min_points_stop)
        self.pub_slow_flag.publish(Bool(slow_flag))
        self.pub_stop_flag.publish(Bool(stop_flag))

        # 좌우 경향
        if left_cnt == 0 and right_cnt == 0 and front_cnt == 0:
            obs_loc = "NONE"
        elif abs(left_cnt - right_cnt) <= 1:
            obs_loc = "CENTER"
        elif left_cnt > right_cnt:
            obs_loc = "LEFT"
        else:
            obs_loc = "RIGHT"
        print(f"obs_loc {obs_loc}")
        self.pub_obs_loc.publish(String(obs_loc))

        # 디버그 이미지 퍼블리시 / 표시
        # if self.pub_img is not None:
        #     img_msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        #     self.pub_img.publish(img_msg)

        if self.use_imshow:
            cv2.imshow("Lidar Rect ROI", canvas)
            cv2.waitKey(1)

def main():
    rospy.init_node("lidar_rect_roi")
    LidarRectROI()
    rospy.spin()
    # 깔끔하게 종료 시 창 닫기
    try:
        cv2.destroyAllWindows()
    except:
        pass

if __name__ == "__main__":
    main()
