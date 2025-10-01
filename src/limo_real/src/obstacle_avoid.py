#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
from time import *


class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub_node")  # 1. node의 이름 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)  # 2. node의 역할 설정
        self.rate = rospy.Rate(100)  # 주기 설정
        self.pub_msg = Twist()  # 메세지 설정
        self.lidar_msg = LaserScan()  # 메세지 설정
        self.lidar_flag = False
        self.total_time = 0
        self.degrees = []

    def run(self):
        if self.lidar_flag == True:
            degree_min = self.lidar_msg.angle_min * 180 / pi
            degree_increment = self.lidar_msg.angle_increment * 180 / pi

            if self.degrees == []:
                self.degrees = [degree_min + degree_increment * index for index, value in enumerate(self.lidar_msg.ranges)]
            obstacles = []
            for index, value in enumerate(self.lidar_msg.ranges):
                if abs(self.degrees[index]) < 75 and 0 < value < 0.5:
                    # print(f"장애물:{[index]")
                    obstacles.append(index)
                else:
                    pass
            print(obstacles)
            if len(obstacles) > 0:
                self.pub_msg.linear.x = 0.1

                left_space = len(self.lidar_msg.ranges) - obstacles[-1]
                left_avg = (len(self.lidar_msg.ranges) + obstacles[-1]) // 2
                right_space = obstacles[0]
                right_avg = obstacles[0] // 2
                if left_space > right_space:
                    avg_index = left_avg
                else:
                    avg_index = right_avg
                self.pub_msg.angular.z = self.degrees[avg_index] * pi / 180
            else:
                self.pub_msg.linear.x = 0.1
                self.pub_msg.angular.z = 0.0
            self.pub.publish(self.pub_msg)  # 3. publish 실행
            self.rate.sleep()  # 5. 주기 실행

    def lidar_cb(self, msg):
        self.lidar_msg = msg
        self.lidar_flag = True


def main():
    pub_class = Pub_class()
    while not rospy.is_shutdown():
        pub_class.run()


if __name__ == "__main__":
    main()
