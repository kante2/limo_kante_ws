#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LimoObstacleDecision:
    """
    (step 방식)
    /limo_solo/target_angle_rad(Float32) 구독
      -> /cmd_vel(Twist) 1회 퍼블리시

    - 전진 속도는 고정(forward_speed)
    - 조향은 angular.z = angular_gain * target_angle_rad
    - 입력이 timeout_sec 이상 안 오면 정지
    """

    def __init__(self):
        # === 최종 cmd_vel 퍼블리셔 ===
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # === 입력: obstacle perception 결과 ===
        # (기존 코드 default 그대로 유지)
        self.in_topic = rospy.get_param("~in_topic", "/limo_solo/target_angle_rad")
        rospy.Subscriber(self.in_topic, Float32, self.cb_target, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

        # === 제어 파라미터 ===
        self.forward_speed = rospy.get_param("~forward_speed", 0.10)
        self.angular_gain  = rospy.get_param("~angular_gain", 1.00)   # rad 기반 게인
        self.timeout_sec   = rospy.get_param("~timeout_sec", 0.50)

        # === 상태 ===
        self.last_target_angle = 0.0
        self.last_msg_time = rospy.Time(0)

        rospy.loginfo("[obstacle] init done (sub=%s -> pub=/cmd_vel)", self.in_topic)

    # ---------------- Callbacks ----------------
    def cb_target(self, msg: Float32):
        self.last_target_angle = msg.data
        self.last_msg_time = rospy.Time.now()

    def shutdown_hook(self):
        rospy.loginfo("[obstacle] shutdown... stop robot")
        self.cmd_pub.publish(Twist())

    # ---------------- obstacle_step ----------------
    def mission_obstacle_step(self):
        """
        한 번 호출할 때마다 현재 상태 기준으로 /cmd_vel 1회 퍼블리시
        (메인 MissionManager가 주기적으로 불러줌)
        """
        cmd = Twist()

        now = rospy.Time.now()
        dt = (now - self.last_msg_time).to_sec() if self.last_msg_time != rospy.Time(0) else 1e9

        # 입력이 끊기면 안전 정지
        if dt > self.timeout_sec:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(1.0, f"[obstacle] target timeout ({dt:.2f}s) -> STOP")
        else:
            cmd.linear.x  = self.forward_speed
            cmd.angular.z = self.angular_gain * self.last_target_angle

            # 필요하면 조향 포화
            # cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))

        self.cmd_pub.publish(cmd)
