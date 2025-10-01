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
        yellow_lower = np.array([10, 60, 40])
        yellow_upper = np.array([50, 255, 255])
        yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
        yellow_bit_img = cv2.bitwise_and(cv_img, cv_img, mask=yellow_mask)
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([179, 60, 255])
        white_mask = cv2.inRange(hsv_img, white_lower, white_upper)
        white_bit_img = cv2.bitwise_and(cv_img, cv_img, mask=white_mask)
        combined_img = cv2.bitwise_or(yellow_bit_img, yellow_bit_img)
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
        matrix = cv2.getPerspectiveTransform(srcs, dsts)
        cv2.line(combined_img, src1, src1, (255, 0, 0), 25)
        cv2.line(combined_img, src2, src2, (0, 255, 0), 25)
        cv2.line(combined_img, src3, src3, (0, 0, 255), 25)
        cv2.line(combined_img, src4, src4, (0, 255, 255), 25)
        warp_img = cv2.warpPerspective(combined_img, matrix, [x, y])
        gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray_img)
        bin_img[gray_img > 0] = 1

        # print(bin_img.shape)
        histogram = np.sum(bin_img, axis=0)
        # print(histogram)
        histogram[histogram < 30] = 0
        left_lane = histogram[0 : x // 2]
        right_lane = histogram[x // 2 : x]
        try:
            left_indices = np.nonzero(left_lane)[0]
            left_lane_avg = (left_indices[0] + left_indices[-1]) // 2
        except:
            left_lane_avg = 80

        try:
            right_indices = np.nonzero(right_lane)[0]
            right_lane_avg = (right_indices[0] + right_indices[-1]) // 2
            right_lane_avg = right_lane_avg + 320
        except:
            right_lane_avg = x - 80

        center_avg = (left_lane_avg + right_lane_avg) // 2
        print(left_lane_avg)
        self.pub_msg.angular.z = (120 - left_lane_avg) * pi / 640
        self.pub_msg.angular.z = self.pub_msg.angular.z * 2
        self.pub_msg.linear.x = 0.2
        self.pub.publish(self.pub_msg)
        cv2.line(cv_img, [320, 0], [320, y], [0, 255, 0], 5)
        cv2.line(cv_img, [center_avg, 0], [center_avg, y], [0, 255, 255], 5)
        # cv2.line(cv_img, [right_lane, 40], [right_lane, 40], [0, 255, 255], 5)
        cv2.line(warp_img, [left_lane_avg, 440], [left_lane_avg, 440], [0, 0, 255], 20)
        cv2.imshow("cv_img", cv_img)
        cv2.imshow("white_mask", white_mask)
        cv2.imshow("combined_img", combined_img)
        cv2.imshow("hsv_img", hsv_img)
        cv2.imshow("warp_img", warp_img)
        cv2.imshow("gray_img", gray_img)
        cv2.imshow("bin_img", bin_img)
        cv2.waitKey(1)


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()
