class CameraNode:
    def __init__(self):
        pass#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '../..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 
import json
from std_msgs.msg import String
from drive_perception.camera.preprocessing import CameraPreprocessor
from drive_perception.camera.feature_extraction import LaneFeatureExtractor
from drive_perception.camera.sliding_window import SlidingWindow
from utills import check_timer

class PerCamera:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("per_camera_node")
        print(f"PerCamera start")
        self.init_pubSub()
        self.init_msg()
        self.init_processing()
        self.init_timer()

    def init_pubSub(self):
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.CB_cam_raw, queue_size=1) # // 
        self.pub = rospy.Publisher('/perception/camera', String, queue_size=1)
    def init_msg(self):
        self.img = None
        self.bridge = CvBridge()
    def init_processing(self):
        self.CameraPreprocessor = CameraPreprocessor()
        self.LaneFeatureExtractor = LaneFeatureExtractor()
        self.SlidingWindow = SlidingWindow()
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("per_camera_node")

    def CB_cam_raw(self,msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.processing()

    def pub_cam_info(self,dataset):
        json_str = json.dumps(dataset)
        self.pub.publish(json_str)
        
    def view_cam(self):
        combined_img = cv2.bitwise_or(self.white_lane_img,self.yellow_lane_img)
        if self.stop_line != []:
            cross_threshold = 35
            min_y, max_y = self.stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                cv2.rectangle(combined_img,[0,min_y],[self.img_x, max_y],[0,0,255],3)
        #cv2.imshow("white_lane_img",white_lane_img)
        cv2.imshow("lane_img",combined_img)
        # cv2.imshow("self.img",self.img)
        cv2.waitKey(1)
        # end = time()
        # print(f"time1 {end - start1} ")
        
    def processing(self):
        try:
            self.img_y, self.img_x = self.img.shape[0:2]
            warped_img = self.CameraPreprocessor.BEV_img_warp(self.img,self.img_y,self.img_x)
            warped_img_hsv = cv2.cvtColor(warped_img,cv2.COLOR_BGR2HSV)
            yellow_filtered_img, white_filtered_img = self.CameraPreprocessor.detect_color_yAndw(warped_img,warped_img_hsv)
            yellow_bin_img,white_bin_img = self.CameraPreprocessor.img_binary_yAndw(yellow_filtered_img, white_filtered_img)

            self.SlidingWindow.set_img_y(self.img_y)
            self.lane_mode = self.LaneFeatureExtractor.estimate_lane_mode(warped_img)
            self.stop_line, white_bin_img= self.LaneFeatureExtractor.estimate_stop_line(white_bin_img,self.img_y)
            self.yellow_lane_img, yellow_left_lane, yellow_right_lane = self.SlidingWindow.sliding_window_adaptive(yellow_bin_img)
            self.white_lane_img, white_left_lane, white_right_lane = self.SlidingWindow.sliding_window_adaptive(white_bin_img)
            
            dataset = [self.stop_line, yellow_left_lane, yellow_right_lane,white_left_lane, white_right_lane,self.lane_mode]           
            self.pub_cam_info(dataset)
            
            self.view_cam()
        except Exception as e:
            pass
     
        
if __name__ == '__main__':
    node = PerCamera()
    rospy.spin()
    