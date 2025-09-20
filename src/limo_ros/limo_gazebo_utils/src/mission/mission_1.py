#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import os
import rospy
from time import sleep

# ---- 프로젝트 의존 ----
# from drive_decision.ctrl import ctrl_motor_servo
from limo_gazebo_utils_limo_control import ctrl_motor_servo
# from drive_decision.lane import dec_lane_curvature
from limo_gazebo_utils.limo_decision import dec_lane_curvature 

MAX_Y = 1
SKIP_STOPLINE = 3

class DecLaneMode_000:
    def __init__(self, CtrlMotorServo, DecLaneCurvature):
        self.init_mission2_3()
        self.init_processing(CtrlMotorServo, DecLaneCurvature)
        
    def init_mission2_3(self):
        # 공통 상태
        self.front_near_obstacle = False
        self.lane_mode     = "right"
        self.avoid_side       = None
        self.in_avoid_mode    = False
        self.dynamic_obs_flag = False
        self.obs_flag         = False
        self.count_stopsline  = 0

        # 정지선 상태 (비블로킹)
        self.stopline_seen_once   = False   # "봤다" 상태(사라지기 전까지 재트리거 방지)
        self.stopline_active      = False   # 현재 정지선 명령 유지 중?
        self.stopline_until       = 0.0     # 유지 종료 시각
        self.stopline_cmd         = (0.5, 0)  # (steer, speed)
        self.stopline_threshold_y = 50

        # mission mode 변경
        self.check_finish_mode2_flag = False
        self.check_time_finish_flag = rospy.get_time()
        self.check_time_prev = rospy.get_time()
        # 회피 상태
        self.waypoint_idx   = -1
        self.last_time      = 0.0
        self.sleep_duration = 0.0
        
        # 플랜 (예: 정지선 처리용)
        self.thick_plan = {
            # 첫 번째 정지선 + 현재 차선이 left → thick_plan의 insert_right_line 강제
            "insert_right_line": {"type": "steer_fixed", "steer": 0.7, "speed": 800, "duration": 1.0},

            # 2차선 
            1: {"type": "steer_fixed", "steer": 0.5, "speed": 800, "duration": 1.5},
            2: {"type": "steer_fixed", "steer": 0.5, "speed": 800, "duration": 1.5},
            # 3: {"type": "steer_fixed", "steer": 0.5, "speed": 0, "duration": 0.0},
            # 3번쨰에서 딜레이 처리로 개수를 한번 처리해야되는데 DURATION 이 0이여서 2번 카운트됨#
            3: {"type": "steer_fixed", "steer": 0.5, "speed": 300, "duration": 1.0},
            4: {"type": "steer_fixed", "steer": 0.5, "speed": 800, "duration": 1.0},
            "default" : {"type": "steer_fixed", "steer": 0.5, "speed": 1000, "duration": 1.5}
            # "default" : {"type": "steer_fixed", "steer": 1.0, "speed": 1000, "duratiwon": 0.0}
        }
    def init_processing(self, CtrlMotorServo: ctrl_motor_servo.CtrlMotorServo,
                        DecLaneCurvature: dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo   = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature

    def set_front_near_obstacle(self, flag: bool):
        self.front_near_obstacle = flag
    def set_lane_mode(self, lane: str):
        self.lane_mode = lane
    def set_dynamic_obs_flag(self, flag: bool):
        self.dynamic_obs_flag = flag

    # ---------------------------
    # 정지선 트리거(비블로킹: 상태만 세팅)
    # ---------------------------
    def is_stopLine(self,stop_line):
        return isinstance(stop_line, (list, tuple)) and len(stop_line) > MAX_Y
    def is_over_stopLine_threshold(self,y_val):
        return y_val is not None and y_val > self.stopline_threshold_y
    def is_over_stopLine_until_time(self,now):
        return self.stopline_active and now >= self.stopline_until
    def is_not_seen_stopLine_first(self):
        return not self.stopline_seen_once and not self.stopline_active
    def try_stopline_action(self, stop_line, now: float) -> None:
        y_val = None
        if self.is_stopLine(stop_line):
            y_val = stop_line[MAX_Y]
            # rospy.loginfo(f"[DBG] stop_line_y={y_val}")
        else:
            # rospy.loginfo(f"[DBG] stop_line 구조가 예상과 다릅니다: {stop_line}")
            pass

        if self.is_over_stopLine_threshold(y_val): # 정지선 처리 부분
            if self.is_not_seen_stopLine_first():
                # 이번에 적용할 회차 = 지금까지 완료한 수 + 1
                next_idx = self.count_stopsline + 1
                lane_lower = str(self.lane_mode).lower()
                print(f" --------------------------------- self.lane_mode {self.lane_mode}")
                print(f"---------------------------------lane_lower {lane_lower}")
                print(f" --------------------------------- NEXT INDEX / 이번회차  {next_idx}")
                # ✅ 규칙: 정지선 + 현재 차선이 left → thick_plan[1] 강제
                if lane_lower == "left":
                    plan = self.thick_plan.get("insert_right_line")
                    chosen_key = "insert_right_line"
                # 정지선 + 현재 차선이 right -> next_idx, 업데이트 한 두꺼운선 기준
                else:
                    # 기본 선택 로직 (회차키 → 문자열키 → default)
                    plan = (
                        self.thick_plan.get(next_idx)
                        or self.thick_plan.get(str(next_idx))
                        or self.thick_plan.get("default")
                    )
                    chosen_key = next_idx if plan is not None else "default"

                rospy.loginfo(
                    f"[STOPLINE] lane={self.lane_mode}, next_idx={next_idx}, chosen_key={chosen_key}, keys={list(self.thick_plan.keys())}"
                )

                if plan and plan.get("type") == "steer_fixed":
                    steer    = float(plan["steer"])
                    speed    = float(plan["speed"])
                    duration = float(plan["duration"])

                    self.stopline_cmd    = (steer, speed)
                    self.stopline_active = True
                    self.stopline_until  = now + duration
                    self.count_stopsline += 1

                    rospy.loginfo(
                        f"[STOPLINE] start #{next_idx} (key={chosen_key}): y={y_val}, keep {duration}s, "
                        f"steer={steer}, speed={speed}, count={self.count_stopsline}"
                    )
                else:
                    rospy.logwarn(f"[STOPLINE] thick_plan에 key={chosen_key} 또는 'default' 설정이 없습니다.")
            self.stopline_seen_once = True
        else:
            self.stopline_seen_once = False

        if self.is_over_stopLine_until_time(now):
            self.stopline_active = False
            rospy.loginfo("[STOPLINE] end")

    # ---------------------------
    # 회피 상태 업데이트(비블로킹)
    # ---------------------------
    def is_front_obstacle(self):
        return self.front_near_obstacle or self.in_avoid_mode
    def is_setting_avoide_mode(self):
        return not self.in_avoid_mode
    def is_checking_next_motion(self,now):
        return now - self.last_time > self.sleep_duration
    def FSM_avoide_00(self):
        return self.waypoint_idx == 0
    def FSM_avoide_01(self):
        return self.waypoint_idx == 1
    def FSM_avoide_02(self):
        return self.waypoint_idx == 2
    def FSM_avoide_03(self):
        return self.waypoint_idx == 3
    def update_avoidance(self, now: float):
        if self.is_front_obstacle():
            # print(f"front_near_obstacle")
            if self.is_setting_avoide_mode():
                self.avoid_side     = "left" if self.lane_mode == "right" else "right"
                self.in_avoid_mode  = True
                self.obs_flag       = True
                self.waypoint_idx   = 0
                self.last_time      = now
                self.sleep_duration = 1.0

            if self.is_checking_next_motion(now):
                self.waypoint_idx += 1
                self.last_time = now

            if self.FSM_avoide_00():
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 1400
                self.sleep_duration = 0.15

            elif self.FSM_avoide_01():
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 900
                self.sleep_duration = 0.55

            elif self.FSM_avoide_02():
                steer, speed = (0.9 if self.avoid_side == "left" else 0.1), 1200
                self.sleep_duration = 0.6

            elif self.FSM_avoide_03():
                steer, speed = 0.5, 600
                self.sleep_duration = 0.1

            elif self.waypoint_idx == 4:
                self.in_avoid_mode = False
                self.front_near_obstacle = False
                self.obs_flag   = False
                self.waypoint_idx = -1
                self.sleep_duration= 0.0
                self.avoid_side   = None
                steer, speed = 0.5, 800
            else:
                self.in_avoid_mode = False
                self.front_near_obstacle = False
                self.obs_flag   = False
                self.waypoint_idx = -1
                steer, speed = 0.5, 0

            return (steer, speed), True
        return (None, None), False

 # ---------------------------
    # 메인: 병렬 감지 + 우선순위 합성
    # --------------------------
    def is_detect_dynamic_obs_far(self):
        return self.dynamic_obs_flag == "slow_flag"
    def is_detect_dynamic_obs_near(self):
        return self.dynamic_obs_flag == "stop_flag"
    def is_move_stopLine_sequence(self):
        return self.stopline_active and (self.count_stopsline == 1 or
                                         self.count_stopsline == 2 or self.count_stopsline == 4)
    def is_avoiding_dynamic_obs(self,avoid_on):
        return avoid_on
    def is_setting_changing_mission_mode(self):
        return self.count_stopsline == 4 and not self.check_finish_mode2_flag
    def is_checking_changing_mission_mode(self,now):
        return self.check_finish_mode2_flag and self.check_time_finish_flag < now
    def handle_zone_mission2_3(self, stop_line):
        # print(f"self.lane_mode {self.lane_mode}")
        now = rospy.get_time()

        # 병렬 업데이트 (블로킹 없음)
        self.try_stopline_action(stop_line, now)
        (avoid_cmd, avoid_on) = self.update_avoidance(now)

        # 우선순위: 동적장애물 > 회피 > 정지선 > 차선주행

        # <-----------장애물 회피-------------->
        # 1) 동적 장애물 → 감속/정지 (최우선, 바로 publish & return)

        if self.is_detect_dynamic_obs_far():
            steer, speed = 0.5, 300
            self.lidar_flag   = False
            self.obs_flag     = True
            self.waypoint_idx = -1
            # publish & return
            self.CtrlMotorServo.pub_move_motor_servo(speed, steer)
            self.check_time_finish_flag += now - self.check_time_prev
            self.check_time_prev = now
            return False

        elif self.is_detect_dynamic_obs_near():
            steer, speed = 0.5, 0
            self.lidar_flag   = False
            self.obs_flag     = True
            self.waypoint_idx = -1
            # [ADD] 1초 정지 유지 예약
            self.stop_hold_until = max(getattr(self, "stop_hold_until", 0.0), now + 1.0)
            # publish & return
            self.CtrlMotorServo.pub_move_motor_servo(speed, steer)
            self.check_time_finish_flag += now - self.check_time_prev
            self.check_time_prev = now
            return False

        if self.is_avoiding_static_obs(avoid_on):
            steer, speed = avoid_cmd
            self.CtrlMotorServo.pub_move_motor_servo(speed, steer)
            return False

        # if self.stopline_active and self.count_stopsline <= 2:
        if  self.is_move_stopLine_sequence():
            steer, speed = self.stopline_cmd # <- 
            self.CtrlMotorServo.pub_move_motor_servo(speed, steer)
            return False

        print(f"self.check_time_finish_flag {self.check_time_finish_flag - now}")
        if self.is_setting_changing_mission_mode():
            self.check_finish_mode2_flag = True
            self.check_time_finish_flag = rospy.get_time() + 9.3 # 9.2
        if self.is_checking_changing_mission_mode(now):
            return True
        # 기본 차선 주행
        self.DecLaneCurvature.decision(SKIP_STOPLINE)
        self.check_time_prev = now
        
        return False
        # mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision()
        # self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)