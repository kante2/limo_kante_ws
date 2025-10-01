#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class CtrlMotorServo:
    def __init__(self):
        # === ROS params (필요시 launch에서 세팅) ===
        self.use_twist      = rospy.get_param("~use_twist", True)  # True면 /limo/cmd_vel 퍼블리시
        self.cmd_vel_topic  = rospy.get_param("~cmd_vel_topic", "/limo/cmd_vel")

        self.motor_topic    = rospy.get_param("~motor_topic", "/commands/motor/speed")
        self.servo_topic    = rospy.get_param("~servo_topic", "/commands/servo/position")

        # 값 스케일링(네 시스템에 맞춰 조정)
        # 현재 너의 컨트롤 값은 speed≈700~1200, steer≈0.0~1.0 근처였음
        self.speed_scale    = rospy.get_param("~speed_scale", 0.001)  # 1200 -> 1.2 m/s 처럼
        self.steer_center   = rospy.get_param("~steer_center", 0.5)   # 0.5가 직진
        self.steer_gain     = rospy.get_param("~steer_gain", 2.0)     # (steer-0.5)*2.0 -> rad/s

        self.init_pub()

    def init_pub(self):
        if self.use_twist:
            self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        else:
            self.motor_pub = rospy.Publisher(self.motor_topic, Float64, queue_size=1)
            self.servo_pub = rospy.Publisher(self.servo_topic, Float64, queue_size=1)

    def pub_move_motor_servo(self, speed, steer):
        """
        speed: 네 내부 단위(예: 700~1200 PWM or arbitrary)
        steer: 0.0~1.0에서 0.5=직진 기준으로 들어온다고 가정
        """
        if self.use_twist:
            # 내부 단위를 m/s, rad/s로 변환
            v = float(speed) * self.speed_scale
            w = float(steer - self.steer_center) * self.steer_gain

            msg = Twist()
            msg.linear.x  = v
            msg.angular.z = w
            self.cmd_pub.publish(msg)
        else:
            speed_msg = Float64()
            steer_msg = Float64()
            speed_msg.data = float(speed)
            steer_msg.data = float(steer)
            self.motor_pub.publish(speed_msg)
            self.servo_pub.publish(steer_msg)



# ---------------------------------

# #!/usr/bin/env python3
# #-*- coding:utf-8 -*- 
# import sys
# import os

# # current_dir = os.path.dirname(os.path.abspath(__file__))  # dec_mission_all.py가 있는 폴더
# # # src/drive_decision 까지 상대 경로로 올라갔다가 내려가기 (예: 현재 파일 위치 기준)
# # drive_decision_path = os.path.abspath(os.path.join(current_dir, '..', '..', 'src', 'drive_decision'))

# if drive_decision_path not in sys.path:
#     sys.path.insert(0, drive_decision_path)
    
# # current_dir = os.path.dirname(os.path.abspath(__file__))
# # parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
# # if parent_dir not in sys.path:
# #     sys.path.insert(0, parent_dir)

# from time import *
# import rospy
# from std_msgs.msg import Float64

# class CtrlMotorServo:
#     def __init__(self):
#         self.init_pubSub()
        
#     def init_pubSub(self):
#         self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
#         self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

#     '''
#     ---
#     rostopic echo /limo/cmd_vel
#     linear: 
#     x: 0.25
#     y: 0.0
#     z: 0.0a
#     angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#     ---
#     '''
        
#     def pub_move_motor_servo(self, speed, steer):
#         speed_msg = Float64()
#         steer_msg = Float64()
#         speed_msg.data = speed
#         steer_msg.data = steer
#         self.motor_pub.publish(speed_msg)
#         self.servo_pub.publish(steer_msg)