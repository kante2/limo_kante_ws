#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LidarAvoidPerson:
    def __init__(self,
                 scan_topic="/scan",
                 cmd_topic="/cmd_vel",
                 sector_deg=40.0,        # 전방 ±각도
                 stop_dist=0.60,         # 정지 임계
                 slow_dist=1.00,         # 감속 임계
                 max_speed=0.25,
                 slow_speed=0.10,
                 turn_speed=0.25):
        self.sector_deg  = float(sector_deg)
        self.stop_dist   = float(stop_dist)
        self.slow_dist   = float(slow_dist)
        self.max_speed   = float(max_speed)
        self.slow_speed  = float(slow_speed)
        self.turn_speed  = float(turn_speed)

        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.CB_scan, queue_size=1)

        self.last_turn_left = True  # 히스테리시스용

    def _indices_for_sector(self, scan: LaserScan, center_rad=0.0, half_width_deg=20.0):
        """지정된 각 섹터(라디안 center, 반폭 deg)에 해당하는 인덱스 범위 계산"""
        half = math.radians(half_width_deg)
        a_min, a_inc = scan.angle_min, scan.angle_increment
        a_max = scan.angle_max
        left  = max(a_min, center_rad - half)
        right = min(a_max, center_rad + half)
        i_left  = int(max(0, math.floor((left  - a_min)/a_inc)))
        i_right = int(min(len(scan.ranges)-1, math.ceil((right - a_min)/a_inc)))
        return i_left, i_right

    def _min_in_sector(self, scan: LaserScan, i0: int, i1: int):
        vals = [r for r in scan.ranges[i0:i1+1] if not math.isinf(r) and not math.isnan(r)]
        return min(vals) if vals else float("inf")

    def CB_scan(self, scan: LaserScan):
        # 전방 섹터(±sector_deg)
        iL, iR = self._indices_for_sector(scan, 0.0, self.sector_deg)
        d_center = self._min_in_sector(scan, iL, iR)

        # 좌/우 섹터(회피 방향 결정용)
        iLL, iLR = self._indices_for_sector(scan,  math.radians(+45), self.sector_deg/2.0)
        iRL, iRR = self._indices_for_sector(scan,  math.radians(-45), self.sector_deg/2.0)
        d_left  = self._min_in_sector(scan, iLL, iLR)
        d_right = self._min_in_sector(scan, iRL, iRR)

        cmd = Twist()

        if d_center <= self.stop_dist:
            # 아주 가까운 장애물 → 정지 & 회피 회전
            cmd.linear.x = 0.0
            # 더 여유가 있는 쪽으로 회전
            turn_left = d_left > d_right if abs(d_left - d_right) > 0.05 else self.last_turn_left
            cmd.angular.z = +self.turn_speed if turn_left else -self.turn_speed
            self.last_turn_left = turn_left

        elif d_center <= self.slow_dist:
            # 전방에 장애물은 있으나 여유 있음 → 감속 & 약간 회전
            cmd.linear.x  = self.slow_speed
            turn_left = d_left > d_right if abs(d_left - d_right) > 0.05 else self.last_turn_left
            cmd.angular.z = +self.turn_speed*0.6 if turn_left else -self.turn_speed*0.6
            self.last_turn_left = turn_left

        else:
            # 전방 클리어 → 직진
            cmd.linear.x  = self.max_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
