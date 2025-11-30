#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


class LimoLabacornDecision:
    """
    라바콘(갭 네비게이션) 디시전 모듈.
    - 입력: /labacorn/yaw (Float32), /labacorn/valid (Bool)
    - 출력: /cmd_vel (Twist)
    - 메인 루프에서 labacorn_decision_step()을 계속 호출해서 사용.
    """

    def __init__(self):
        # === 내부 상태 ===
        self.latest_yaw = 0.0      # perception에서 받은 목표 yaw [rad]
        self.latest_valid = False  # 타겟 유효 여부

        # === 파라미터 ===
        self.follow_speed_lin = rospy.get_param("~follow_speed_lin", 0.1)    # 정상 속도
        self.min_speed_lin    = rospy.get_param("~min_speed_lin",    0.05)   # 타겟 없을 때 속도
        self.k_yaw            = rospy.get_param("~k_yaw",           -1.4)    # 조향 게인
        self.max_ang_z        = rospy.get_param("~max_ang_z",        1.6)    # 최대 조향 속도
        self.no_target_forward = rospy.get_param("~no_target_forward", True) # 타겟 없을 때 전진/정지

        yaw_topic   = rospy.get_param("~yaw_topic",   "/labacorn/yaw")
        valid_topic = rospy.get_param("~valid_topic", "/labacorn/valid")
        cmd_topic   = rospy.get_param("~cmd_topic",   "/cmd_vel")

        # === Publisher ===
        self.pub_cmd = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        # === Subscriber ===
        rospy.Subscriber(yaw_topic,   Float32, self.yaw_cb,   queue_size=1)
        rospy.Subscriber(valid_topic, Bool,    self.valid_cb, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo(
            "[LimoLabacornDecision] init done "
            "(follow_speed_lin=%.3f, min_speed_lin=%.3f, k_yaw=%.2f, max_ang_z=%.2f)",
            self.follow_speed_lin, self.min_speed_lin, self.k_yaw, self.max_ang_z
        )

    # ---------------- 콜백 ----------------
    def yaw_cb(self, msg: Float32):
        self.latest_yaw = msg.data

    def valid_cb(self, msg: Bool):
        self.latest_valid = msg.data

    def shutdown_hook(self):
        rospy.loginfo("[LimoLabacornDecision] shutdown -> stop robot")
        self.pub_cmd.publish(Twist())

    # ---------------- 헬퍼: 타겟 없을 때 속도 발행 ----------------
    def _publish_minimal_speed(self):
        cmd = Twist()
        if self.no_target_forward:
            cmd.linear.x = self.min_speed_lin
            cmd.angular.z = 0.0
            rospy.loginfo_throttle(1.0, "[labacorn_decision] No target, moving forward slowly")
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.loginfo_throttle(1.0, "[labacorn_decision] No target, stopping")

        self.pub_cmd.publish(cmd)

    # ---------------- 핵심: 한 스텝 ----------------
    def labacorn_decision_step(self):
        """
        메인 루프에서 매 주기마다 호출.
        - latest_yaw, latest_valid 를 사용해서 /cmd_vel 하나 발행
        """
        if not self.latest_valid:
            self._publish_minimal_speed()
            return

        # yaw -> angular.z
        raw = self.latest_yaw * self.k_yaw
        ang_z = max(-self.max_ang_z, min(self.max_ang_z, raw))

        cmd = Twist()
        cmd.linear.x  = self.follow_speed_lin
        cmd.angular.z = ang_z
        self.pub_cmd.publish(cmd)

        yaw_deg = self.latest_yaw * 180.0 / math.pi
        rospy.loginfo_throttle(
            1.0,
            "[labacorn_decision] v=%.3f m/s, ang_z=%.3f rad/s (yaw=%.3f rad=%.1f deg)",
            cmd.linear.x, cmd.angular.z, self.latest_yaw, yaw_deg
        )
