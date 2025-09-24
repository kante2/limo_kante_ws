#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
LidarHardAvoider
- 평상시: 전진 (v_forward)
- /lidar_stop_flag=True 가 들어오면, /lidar_obs_location에 따라 하드코딩 회피 시퀀스 실행
  * LEFT  → 우측 회피(steer 음수 → 오른쪽 회전)
  * RIGHT → 좌측 회피(steer 양수 → 왼쪽 회전)
  * CENTER/NONE → 기본 회피 방향(default_avoid_dir)
- 시퀀스 완료 후 NORMAL 복귀. stop이 계속 True면 다시 재진입 가능.
"""

import time
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

STATE_NORMAL = "NORMAL"
STATE_AVOID  = "AVOID"

class LidarHardAvoider:
    def __init__(self):
        rospy.init_node("lidar_hard_avoider")

        # ---- Topics ----
        self.stop_topic = rospy.get_param("~stop_flag_topic", "/lidar_stop_flag")
        self.loc_topic  = rospy.get_param("~obs_location_topic", "/lidar_obs_location")
        self.cmd_topic  = rospy.get_param("~cmd_topic", "/limo/cmd_vel")

        # ---- Normal driving ----
        self.v_forward  = float(rospy.get_param("~v_forward", 0.20))  # m/s

        # ---- Avoid params ----
        self.default_avoid_dir = rospy.get_param("~default_avoid_dir", "RIGHT").upper()

        # 스텝별 (angular.z=steer, linear.x=speed, sec=duration)
        # 아래 프로파일은 "왼쪽 회피(양수 steer) / 오른쪽 회피(음수 steer)" 기준
        self.avoid_steps_left  = rospy.get_param("~avoid_steps_left",  [  # 장애물 RIGHT → 왼쪽으로 회피
            {"steer": +1.2, "speed": 0.10, "duration": 0.15},
            {"steer": +1.2, "speed": 0.06, "duration": 0.55},
            {"steer": -1.0, "speed": 0.10, "duration": 0.60},
            {"steer":  0.0, "speed": 0.05, "duration": 0.10},
        ])
        self.avoid_steps_right = rospy.get_param("~avoid_steps_right", [  # 장애물 LEFT  → 오른쪽으로 회피
            {"steer": -1.2, "speed": 0.10, "duration": 0.15},
            {"steer": -1.2, "speed": 0.06, "duration": 0.55},
            {"steer": +1.0, "speed": 0.10, "duration": 0.60},
            {"steer":  0.0, "speed": 0.05, "duration": 0.10},
        ])

        # ---- State ----
        self.state       = STATE_NORMAL
        self.stop_flag   = False
        self.obs_loc     = "NONE"      # LEFT/RIGHT/CENTER/NONE
        self.avoid_plan  = []          # 현재 실행할 스텝 리스트
        self.step_idx    = -1
        self.step_until  = 0.0

        # ---- ROS I/O ----
        rospy.Subscriber(self.stop_topic, Bool, self.cb_stop, queue_size=10)
        rospy.Subscriber(self.loc_topic,  String, self.cb_loc,  queue_size=10)
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)

        # 20 Hz 제어 루프
        self.dt = float(rospy.get_param("~ctrl_dt", 0.05))
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.tick)
        rospy.loginfo("LidarHardAvoider started: stop=%s, loc=%s -> cmd=%s",
                      self.stop_topic, self.loc_topic, self.cmd_topic)

    # ---------- Callbacks ----------
    def cb_stop(self, msg: Bool):
        self.stop_flag = bool(msg.data)

    def cb_loc(self, msg: String):
        val = (msg.data or "").upper().strip()
        if val in ("LEFT", "RIGHT", "CENTER", "NONE"):
            self.obs_loc = val

    # ---------- Helpers ----------
    def make_cmd(self, v_lin, w_ang):
        cmd = Twist()
        cmd.linear.x  = max(0.0, float(v_lin))  # 안전 제한
        cmd.angular.z = float(w_ang)
        return cmd

    def choose_avoid_plan(self):
        # LEFT  → 왼쪽에 장애물 → 오른쪽 회피(avoid_steps_right)
        # RIGHT → 오른쪽에 장애물 → 왼쪽 회피(avoid_steps_left)
        side = self.obs_loc
        print(f"obstacle detected _ {side}")
        if side == "LEFT":
            return list(self.avoid_steps_right)
        elif side == "RIGHT":
            return list(self.avoid_steps_left)
        else:
            return list(self.avoid_steps_left if self.default_avoid_dir == "LEFT" else self.avoid_steps_right)

    def enter_avoid(self):
        self.avoid_plan = self.choose_avoid_plan()
        self.state      = STATE_AVOID
        self.step_idx   = -1
        self.step_until = 0.0
        rospy.loginfo("[AVOID] enter: obs_loc=%s, steps=%d", self.obs_loc, len(self.avoid_plan))

    def step_advance(self, now):
        self.step_idx += 1
        if self.step_idx >= len(self.avoid_plan):
            # 종료
            self.state = STATE_NORMAL
            self.avoid_plan = []
            rospy.loginfo("[AVOID] finished → NORMAL")
            return

        step = self.avoid_plan[self.step_idx]
        dur  = float(step.get("duration", 0.3))
        self.step_until = now + max(0.0, dur)
        rospy.loginfo("[AVOID] step %d/%d: steer=%.3f, speed=%.3f, dur=%.2f",
                      self.step_idx+1, len(self.avoid_plan),
                      float(step.get("steer", 0.0)),
                      float(step.get("speed", 0.0)),
                      dur)

    # ---------- Main loop ----------
    def tick(self, _evt):
        now = time.time()

        # NORMAL → AVOID 전이
        if self.state == STATE_NORMAL and self.stop_flag:
            self.enter_avoid()

        if self.state == STATE_AVOID:
            # 스텝 시작/전환
            if now >= self.step_until:
                self.step_advance(now)

            if self.state == STATE_AVOID and 0 <= self.step_idx < len(self.avoid_plan):
                step = self.avoid_plan[self.step_idx]
                steer = float(step.get("steer", 0.0))
                speed = float(step.get("speed", 0.0))
                self.pub_cmd.publish(self.make_cmd(speed, steer))
                return

        # NORMAL: 계속 전진
        self.pub_cmd.publish(self.make_cmd(self.v_forward, 0.0))

def main():
    node = LidarHardAvoider()
    rospy.spin()

if __name__ == "__main__":
    main()
