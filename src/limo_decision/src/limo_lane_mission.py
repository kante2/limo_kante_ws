#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


class LimoLaneDecision:
    def __init__(self):
        # === 최종 cmd_vel 퍼블리셔 ===
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # === 입력: perception 결과 ===
        center_error_topic = rospy.get_param("~center_error_topic", "/lane/center_error")
        lane_valid_topic = rospy.get_param("~lane_valid_topic", "/lane/valid")

        rospy.Subscriber(center_error_topic, Float32, self.center_error_cb)
        rospy.Subscriber(lane_valid_topic, Bool, self.valid_cb)

        rospy.on_shutdown(self.shutdown_hook)

        # 상태 변수
        self.center_error = 0.0
        self.lane_valid = False

        # 제어 파라미터
        self.camera_speed = rospy.get_param("~camera_speed", 0.15)
        self.steer_gain = rospy.get_param("~steer_gain", 0.005)

        rospy.loginfo(
            "Limo Lane Decision 초기화 완료 (center_error_topic=%s, valid_topic=%s)",
            center_error_topic,
            lane_valid_topic,
        )

    # ---------------- Callbacks ----------------
    def center_error_cb(self, msg):
        self.center_error = msg.data

    def valid_cb(self, msg):
        self.lane_valid = msg.data

    def shutdown_hook(self):
        rospy.loginfo("decision 모듈 종료... 로봇 정지.")
        self.cmd_pub.publish(Twist())

    # ---------------- lane_decision_step ----------------
    def mission_lane_step(self):
        """
        한 번 호출할 때마다 현재 상태 기준으로 /cmd_vel 한 번 퍼블리시
        """
        cmd = Twist()

        if self.lane_valid:
            cmd.linear.x = self.camera_speed
            cmd.angular.z = -self.center_error * self.steer_gain
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
