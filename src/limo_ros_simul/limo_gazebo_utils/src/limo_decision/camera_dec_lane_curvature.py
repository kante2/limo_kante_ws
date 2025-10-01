#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
limo_control_path = os.path.abspath(os.path.join(current_dir, '..', 'limo_control'))
if limo_control_path not in sys.path:
    sys.path.insert(0, limo_control_path)

    
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from time import *
import rospy
import numpy as np

# from lane.PIDController import PIDController
# from drive_decision.ctrl import ctrl_motor_servo

# 변경
from control_pid import PIDController
from control_motor import CtrlMotorServo   # 파일 내 클래스명이 CtrlMotorServo 라는 가정


STOP = -1
MISSION_MODE1 = 0
MISSION_MODE3 = 1
SKIP_STOPLINE = 3


class DecLaneCurvature:
    def __init__(self,CtrlMotorServo):
        """_summary_
        현재 방법이 2가지로 나누었기에 함수에 이 이름들을 일단 넣도록 한다.
        방법 1, pt01 - 곡률 계산
        방법 2, pt02 - lane으로 일정 거리 계산 
        """
        print(f"DecLaneCurvature create")
        self.init_lane_data()
        self.init_processing(CtrlMotorServo)

    def init_lane_data(self):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
        
        self.center_index = 0
        self.center_pixel = 320
        
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        
        self.hold_until_ts = 0
        
        self.max_speed = 1200
        self.min_speed = 700

        self.pid = PIDController()
        
        self.sequence_active = False
        self.sequence_start_time = 0
        self.LANE_WIDTH_PIXELS = 260
        
        self.stop_flag_num = 0
        self.goal_stop_line = 0     
    # def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo):
    def init_processing(self, CtrlMotorServo: CtrlMotorServo):
        self.CtrlMotorServo = CtrlMotorServo

    def set_camera_info(self,stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane
    def set_speed(self,min_speed,max_speed):
        self.max_speed = min_speed
        self.min_speed = max_speed

    def get_steer_gain_normal(self, curvature):
        A = 40.0  # 최대 gain
        B = 0.002528 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))
    def get_steer_gain_mission_mode_1(self, curvature):
        A = 30.0  # 최대 gain
        # B = 0.002528 # 0.002528
        B = 0.005 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))
    def get_steer_gain_mission_mode_3(self, curvature):
        A = 40.0  # 최대 gain
        # B = 0.001398 # 0.002528
        B = 0.001350 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))
    def get_base_speed(self, curvature):
        min_speed = self.min_speed
        max_speed = self.max_speed

        # 곡률이 작을수록 (급커브) 속도 ↓ / 곡률이 클수록 (직선) 속도 ↑
        curvature = min(curvature, 10000)  # 과도한 곡률 제한

        # 정규화: 0 (급커브) → 1 (직선)
        norm = curvature / 10000.0
        norm = max(0.0, min(1.0, norm))  # 안정화

        # 보간된 속도 계산
        speed = min_speed + norm * (max_speed - min_speed)
        return int(speed)
    
    def select_steer_gain_by_mode(self,mission_mode, curvature):      
        if mission_mode == MISSION_MODE1:
            return self.get_steer_gain_mission_mode_1(curvature)
        if mission_mode == MISSION_MODE3:
            return self.get_steer_gain_mission_mode_3(curvature)
        else:
            return self.get_steer_gain_normal(curvature)  
    
    def ctrl_move(self,mission_mode, curvature, center_index): # left_lane=None, right_lane=None
        pixel_error = center_index - self.center_pixel
        steer_error = pixel_error * self.steer_per_pixel
        #print(f"left_lane:{left_lane} / right_lane:{right_lane}")
        steer_gain = self.select_steer_gain_by_mode(mission_mode,curvature)
            
        pid_output = self.pid.compute(steer_error)
        steer = steer_gain * pid_output + 0.5
                       
        steer = max(self.min_steer, min(self.max_steer, steer))
        base_speed = self.get_base_speed(curvature)
        deviation = abs(steer - 0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))

        self.CtrlMotorServo.pub_move_motor_servo(speed, steer)

        # rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        # rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        # rospy.loginfo(f"[Curvature] value: {curvature:.2f}")
        # rospy.loginfo(f"[steer_gain] value: {steer_gain:.2f}")

        # 부드러운 steer_gain 계산 함수
          
    def which_lane(self,left_lane, right_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
        weight_left = len(left_lane)
        weight_right = len(right_lane)
        
        if weight_left > weight_right:
            left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
            self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "first_lane", left_lane, []        
        else:
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "second_lane", [], right_lane
    def follow_left_lane(self, left_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "left_guided", left_lane, []
    def follow_right_lane(self, right_lane):
        right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
        self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "right_guided", [], right_lane     
    def forward(self):
        self.center_index = self.center_pixel  # 정중앙
        return "go_straight", [], []
    def move_forward(self,left_lane):
        self.center_index = self.center_pixel  # 정중앙
        return "move_straight", left_lane, []
    
    def ctrl_decision(self):
        stop_flag, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        
        if left_yellow_lane and stop_flag:
            return self.move_forward(left_yellow_lane)
        
        elif left_white_lane and right_lane:
            return self.which_lane(left_white_lane, right_lane)
        
        elif left_yellow_lane and right_lane:
            return self.which_lane(left_yellow_lane, right_lane)
        
        elif left_white_lane and not right_lane:
            return self.follow_left_lane(left_white_lane)

        elif left_yellow_lane and not right_lane:
            return self.follow_left_lane(left_yellow_lane)

        elif right_lane and not left_white_lane:
            return self.follow_right_lane(right_lane)

        else:
            return self.forward()     
    
    def mode_order(self, mode):
        stop_flag, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        left_lane = yellow_left if yellow_left else white_left
        right_lane = white_right
        

        if mode == "move_to_1st_lane":
            self.center_index = 160
            return left_lane, right_lane
        elif mode == "move_to_2nd_lane":
            self.center_index = 480
            return left_lane, right_lane
            
        elif mode == "follow_left_lane":
            if not left_lane:
                right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
                self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                # self.center_index -= 55
                self.center_index -= 50
                # print(f"self.center_index111 {self.center_index}")
            else:
                left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
                self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                self.center_index -= 35
                # print(f"self.center_index222 {self.center_index}")
            return left_lane, []
        elif mode == "follow_right_lane":
            if not right_lane:
                self.center_index = self.center_pixel
            else:
                right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
                self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return [], right_lane
        
        elif mode == "stop":
            self.CtrlMotorServo.pub_move_motor_servo(0,0.5)
            return [], []

    def calculate_curvature(self, left_lane, right_lane):
        x_vals, y_vals = [], []

        if left_lane:
            for pt in left_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        elif right_lane:
            for pt in right_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        # 최소 데이터 개수 확인 (2개 미만이면 근사 자체가 불가능함)
        if len(x_vals) < 5:
            return 1e4  # 데이터 부족 시 직선으로 간주

        try:
            # 2차 다항 근사 (y에 대한 x 곡선으로)
            fit = np.polyfit(y_vals, x_vals, 2)
            A, B = fit[0], fit[1]

            y_eval = np.max(y_vals)  # 일반적으로 이미지 하단 기준

            denominator = abs(2*A)
            if denominator < 1e-4:
                denominator = 1e-4  # 분모 폭발 방지

            curvature = ((1 + (2*A*y_eval + B)**2)**1.5) / denominator

            # 비정상적으로 큰 곡률 제한 (클수록 직선으로 처리)
            if curvature > 10000:
                curvature = 10000

            return curvature

        except Exception as e:
            rospy.logwarn(f"[Curvature Calculation Error] {e}")
            return 1e4  # 계산 실패 시 직선으로 간주

    def decision(self,mission_mode = -2):
        if mission_mode == STOP:
            self.mode_order("stop")
            return
        elif mission_mode == MISSION_MODE1:
            left_lane, right_lane = self.mode_order("follow_left_lane")
        elif mission_mode == SKIP_STOPLINE:
            state, left_lane, right_lane = self.ctrl_decision()
        else:
            state, left_lane, right_lane = self.ctrl_decision()
            print(f"state {state}")
            if state == "move_straight":
                self.CtrlMotorServo.pub_move_motor_servo(2400,0.5)
                rospy.sleep(1.6)
                return
            
        if mission_mode == MISSION_MODE3:
            self.center_index += 50
            
        curvature = self.calculate_curvature(left_lane,right_lane)
        # print(f"left_lane, right_lane {left_lane, right_lane}")
        self.ctrl_move(mission_mode, curvature, self.center_index)

        # rospy.loginfo(f"weight_left : {left_lane} / weight_right : {right_lane}")