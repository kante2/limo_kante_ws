#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from math import *


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node의 이름 설정
        sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_cb)  # 2. node의 역할 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        self.bridge = CvBridge()
        self.pub_msg = Twist()  # 메세지 설정

    def camera_cb(self, msg):  # 3. callback 실행
        # print(msg)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x, channel = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)
        
        # 노란색 차선 검출
        yellow_lower = np.array([10, 60, 40])
        yellow_upper = np.array([50, 255, 255])
        yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
        yellow_bit_img = cv2.bitwise_and(cv_img, cv_img, mask=yellow_mask)
        
        # 흰색 차선 검출
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([179, 60, 255])
        white_mask = cv2.inRange(hsv_img, white_lower, white_upper)
        white_bit_img = cv2.bitwise_and(cv_img, cv_img, mask=white_mask)
        
        # 노란색과 흰색 차선 결합 (수정됨)
        combined_img = cv2.bitwise_or(yellow_bit_img, white_bit_img)
        
        # 원근 변환을 위한 좌표 설정
        margin_x1 = 0
        margin_x2 = 190
        margin_y = 330
        src1 = (margin_x1, y)
        src2 = (margin_x2, margin_y)
        src3 = (x - margin_x2, margin_y)
        src4 = (x - margin_x1, y)
        srcs = np.float32([src1, src2, src3, src4])
        
        dst1 = (80, y)
        dst2 = (80, 0)
        dst3 = (x - 80, 0)
        dst4 = (x - 80, y)
        dsts = np.float32([dst1, dst2, dst3, dst4])
        
        # 원근 변환 매트릭스 계산
        matrix = cv2.getPerspectiveTransform(srcs, dsts)
        matrix_inv = cv2.getPerspectiveTransform(dsts, srcs)
        
        # 원근 변환 적용 (수정됨: 튜플 사용)
        warp_img = cv2.warpPerspective(combined_img, matrix, (x, y))
        origin_warp_img = cv2.warpPerspective(cv_img, matrix, (x, y))
        
        # 이진화 처리
        gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray_img)
        bin_img[gray_img != 0] = 1
        
        # 슬라이딩 윈도우 설정
        window_n = 8
        window_h = y // 8  # 60
        window_margin = x // 8  # 640/8 = 80
        left_avg_indices = []
        right_avg_indices = []

        # 슬라이딩 윈도우로 차선 검출
        for i in range(0, window_n):
            window_top = y - (i + 1) * window_h
            window_bottom = y - i * window_h
            window = bin_img[window_top:window_bottom, :]
            histogram = np.sum(window, axis=0)
            histogram[histogram < 20] = 0

            left_hist = histogram[0 : x // 2]
            right_hist = histogram[x // 2 : x]
            
            # 왼쪽 차선 검출
            try:
                left_index = np.nonzero(left_hist)[0]
                print(f"{i}:{left_index}")
                left_avg_index = (left_index[0] + left_index[-1]) // 2
                cv2.rectangle(warp_img, (left_avg_index - window_margin, window_top), (left_avg_index + window_margin, window_bottom), [0, 0, 255], 5)
                left_avg_indices.append(left_avg_index)
            except:
                print(f"non {i} left")

            # 오른쪽 차선 검출
            try:
                right_index = np.nonzero(right_hist)[0]
                right_avg_index = (right_index[0] + right_index[-1]) // 2
                right_avg_index = right_avg_index + 320
                cv2.rectangle(warp_img, (right_avg_index - window_margin, window_top), (right_avg_index + window_margin, window_bottom), [255, 0, 0], 5)
                right_avg_indices.append(right_avg_index)
            except:
                print(f"non {i} right")

        # 차선 중심 계산 (수정됨: 예외 처리 추가)
        try:
            # 빈 리스트 확인 후 평균 계산
            if len(left_avg_indices) > 0:
                left_avg = np.average(left_avg_indices)
            else:
                left_avg = 160  # 기본값: 화면 왼쪽 1/4 지점
                print("No left lane detected, using default")
                
            if len(right_avg_indices) > 0:
                right_avg = np.average(right_avg_indices)  # 수정됨: right_avg_indices
            else:
                right_avg = 480  # 기본값: 화면 오른쪽 3/4 지점
                print("No right lane detected, using default")
                
            center_avg = int((left_avg + right_avg) // 2)
            print(f"Left: {left_avg:.1f}, Right: {right_avg:.1f}, Center: {center_avg}")
            
        except Exception as e:
            print(f"Average calculation error: {e}")
            center_avg = 320  # 화면 중앙값으로 기본 설정

        # 로봇 제어 명령 계산 및 발행
        self.pub_msg.angular.z = (320 - center_avg) * pi / 640
        self.pub_msg.angular.z = self.pub_msg.angular.z * 2
        self.pub_msg.linear.x = 0.2
        self.pub.publish(self.pub_msg)
        
        # 시각화
        cv2.line(warp_img, (320, 0), (320, y), [0, 255, 0], 5)  # 수정됨: 튜플 사용
        cv2.line(warp_img, (center_avg, 0), (center_avg, y), [0, 255, 255], 5)  # 수정됨: 튜플 사용
        warp_inv_img = cv2.warpPerspective(warp_img, matrix_inv, (x, y))  # 수정됨: 튜플 사용

        # 이미지 출력
        cv2.imshow("warp_img", warp_img)
        cv2.imshow("cv_img", cv_img)
        cv2.imshow("white_mask", white_mask)
        cv2.imshow("combined_img", combined_img)
        cv2.imshow("gray_img", gray_img)
        cv2.imshow("bin_img", bin_img)
        cv2.imshow("warp_inv_img", warp_inv_img)
        cv2.waitKey(1)


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()