#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class LimoSoloPerception:
    """
    [PERCEPTION 역할]
    - /scan(LaserScan)에서 전방 FOV(-front_fov~+front_fov)만 보고
    - range > open_dist 인 "연속 구간(gap)"을 찾는다
    - 가장 좋은 gap(연속 길이 + 거리 기반 점수)의 중앙각을 target_angle_rad로 계산
    - /limo_solo/target_angle_rad(Float32) 퍼블리시
    - 장애물이 의미 있게 존재하는지(/obstacle/valid)도 퍼블리시 (전방 가까운 값 기준)
    """

    def __init__(self):
        rospy.init_node("limo_solo_perception")

        # -------- Topics --------
        self.scan_topic  = rospy.get_param("~scan_topic", "/scan")
        self.out_topic   = rospy.get_param("~out_topic", "/limo_solo/target_angle_rad")
        self.valid_topic = rospy.get_param("~valid_topic", "/obstacle/valid")

        # -------- Parameters (튜닝 핵심) --------
        # 전방만 보도록 제한 (deg)
        self.front_fov_deg = rospy.get_param("~front_fov_deg", 70.0)

        # "열려있다"로 판단할 거리 (m): 이보다 멀면 gap 후보
        self.open_dist = rospy.get_param("~open_dist", 0.80)

        # gap으로 인정할 최소 연속 bin 개수
        self.min_gap_bins = rospy.get_param("~min_gap_bins", 12)

        # 장애물 존재(valid) 판단 거리 (m): 전방 가까우면 True
        self.obstacle_dist = rospy.get_param("~obstacle_dist", 0.50)

        # fallback: gap이 안 잡히면(전방 거의 막힘) 최대거리 방향을 사용하되,
        # 그래도 애매하면 약간 회전하며 탐색하기 위한 기본값
        self.search_turn_rad = rospy.get_param("~search_turn_rad", 0.7)

        # -------- Pub/Sub --------
        self.pub_target = rospy.Publisher(self.out_topic, Float32, queue_size=1)
        self.valid_pub  = rospy.Publisher(self.valid_topic, Bool, queue_size=1)
        self.sub_scan   = rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)

        rospy.loginfo(
            "[perception] sub=%s -> target_pub=%s, valid_pub=%s (front_fov=%.1fdeg, open_dist=%.2fm)",
            self.scan_topic, self.out_topic, self.valid_topic, self.front_fov_deg, self.open_dist
        )

    def cb_scan(self, scan: LaserScan):
        target_angle, has_obstacle = self.compute_target_angle_rad(scan)
        self.pub_target.publish(Float32(data=float(target_angle)))
        self.valid_pub.publish(Bool(data=bool(has_obstacle)))

    def compute_target_angle_rad(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return 0.0, False

        # NaN/inf/<=0 처리: inf는 "멀다"로 보고 range_max로 치환
        rmax = float(scan.range_max) if scan.range_max > 0 else 10.0
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(np.isinf(ranges), rmax, ranges)
        ranges = np.where(ranges <= 0.0, 0.0, ranges)

        # 전방 FOV 인덱스 범위 계산: angle=0 근처가 전방이라고 가정
        fov = math.radians(self.front_fov_deg)
        i0 = self.angle_to_index(scan, -fov)
        i1 = self.angle_to_index(scan, +fov)
        if i0 > i1:
            i0, i1 = i1, i0
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))

        front_ranges = ranges[i0:i1 + 1]
        front_angles = scan.angle_min + scan.angle_increment * np.arange(i0, i1 + 1)

        # 전방 장애물 존재 여부(valid) 판단: 전방에서 obstacle_dist 이내가 있으면 True
        # (너의 미션에서 "장애물 미션이 유효하다"를 알리는 용도)
        near_mask = (front_ranges > 0.0) & (front_ranges < self.obstacle_dist)
        has_obstacle = bool(np.any(near_mask))

        # -------- gap(연속 free 구간) 찾기 --------
        # free: 충분히 멀다(open_dist 이상)
        free = front_ranges >= float(self.open_dist)

        segments = []
        start = None
        for i, ok in enumerate(free):
            if ok and start is None:
                start = i
            elif (not ok) and start is not None:
                end = i - 1
                segments.append((start, end))
                start = None
        if start is not None:
            segments.append((start, len(free) - 1))

        best_score = -1.0
        best_center_idx = None

        for (s, e) in segments:
            length = (e - s + 1)
            if length < int(self.min_gap_bins):
                continue
            seg_ranges = front_ranges[s:e + 1]
            # 점수: (연속 길이) * (평균거리)  -> 길고 멀수록 좋음
            mean_r = float(np.mean(seg_ranges))
            score = float(length) * mean_r
            if score > best_score:
                best_score = score
                best_center_idx = (s + e) // 2

        if best_center_idx is not None:
            target_angle = float(front_angles[best_center_idx])
            return target_angle, has_obstacle

        # -------- gap이 없으면 fallback --------
        # 전방 FOV에서 "가장 먼 방향"이라도 택한다
        # (완전 막힘이면 최대값도 작을 수 있음)
        max_i = int(np.argmax(front_ranges))
        max_r = float(front_ranges[max_i])

        if max_r <= 1e-3:
            # 전방이 거의 다 0(센서 이상/완전 밀착) -> 탐색 회전값
            return float(self.search_turn_rad), True

        target_angle = float(front_angles[max_i])
        return target_angle, has_obstacle

    @staticmethod
    def angle_to_index(scan: LaserScan, angle_rad: float) -> int:
        # index = (angle - angle_min)/increment
        if scan.angle_increment == 0:
            return 0
        return int(round((angle_rad - scan.angle_min) / scan.angle_increment))


def main():
    _ = LimoSoloPerception()
    rospy.spin()


if __name__ == "__main__":
    main()
