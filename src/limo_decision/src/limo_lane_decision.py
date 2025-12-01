#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool


class LimoLaneDecision:
    def __init__(self):
        # 여기서는 init_node 안 함 (메인에서)
        # rospy.init_node("limo_lane_decision_node")

        # === 최종 cmd_vel 퍼블리셔 ===
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # === 입력: perception & V2X ===
        rospy.Subscriber("/lane/steer", Float32, self.steer_cb)
        rospy.Subscriber("/lane/valid", Bool, self.valid_cb)
        rospy.Subscriber("/path_", String, self.v2x_cb)

        rospy.on_shutdown(self.shutdown_hook)

        # 상태 변수
        self.steer = 0.0
        self.lane_valid = False
        self.v2x = "D"   # 기본: 직진 느낌

        # 기본 속도
        self.camera_speed = rospy.get_param("~camera_speed", 0.15)

        rospy.loginfo("Limo Lane Decision 초기화 완료.")

    # ---------------- Callbacks ----------------
    def steer_cb(self, msg):
        self.steer = msg.data

    def valid_cb(self, msg):
        self.lane_valid = msg.data

    def v2x_cb(self, msg):
        if msg is not None:
            self.v2x = msg.data
            rospy.loginfo(f"V2X 수신: {self.v2x}")

    def shutdown_hook(self):
        rospy.loginfo("decision 모듈 종료... 로봇 정지.")
        self.cmd_pub.publish(Twist())

    # ---------------- lane_decision_step ----------------
    def mission_lane_step(self):
        """
        한 번 호출할 때마다 현재 상태 기준으로 /cmd_vel 한 번 퍼블리시
        """
        cmd = Twist()

        # 기본 로직:
        # - 차선 valid: lane follower
        # - invalid: 정지
        if self.lane_valid:
            cmd.linear.x = self.camera_speed
            cmd.angular.z = self.steer

            # TODO: self.v2x 값에 따라 모드 분기 가능
            # if self.v2x == "STOP": cmd.linear.x = 0.0
            # if self.v2x == "LEFT": cmd.angular.z += 0.5
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
