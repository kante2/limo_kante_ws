#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class LimoSoloPerception:
    """
    /scan(LaserScan) 구독
      -> 가까운 장애물(거리<th) 인덱스를 기반으로 left/right/center 중 가장 큰 빈 공간 방향을 선택
      -> 그 방향의 목표 각도(rad)를 /limo_solo/target_angle_rad(Float32)로 퍼블리시
    """

    def __init__(self):
        rospy.init_node("limo_solo_perception")

        # -------- Parameters --------
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.out_topic = rospy.get_param("~out_topic", "/limo_solo/target_angle_rad")
        self.valid_topic = rospy.get_param("~valid_topic", "/obstacle/valid")

        # 장애물 판단 거리 (m)
        self.obstacle_dist = rospy.get_param("~obstacle_dist", 0.5)

        # 장애물 인덱스 사이가 이 값보다 크면 "중앙 통로"로 간주 (인덱스 간격 기준)
        self.between_obstacle = rospy.get_param("~between_obstacle", 10)

        # -------- Pub/Sub --------
        self.pub_target = rospy.Publisher(self.out_topic, Float32, queue_size=1)
        self.valid_pub = rospy.Publisher(self.valid_topic, Bool, queue_size=1)
        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)

        rospy.loginfo(
            "[perception] sub=%s -> target_pub=%s, valid_pub=%s",
            self.scan_topic,
            self.out_topic,
            self.valid_topic,
        )

    def cb_scan(self, scan: LaserScan):
        target_angle, has_obstacle = self.compute_target_angle_rad(scan)  # rad
        self.pub_target.publish(Float32(data=target_angle))
        self.valid_pub.publish(Bool(data=has_obstacle))

    def compute_target_angle_rad(self, scan: LaserScan):
        """
        원 코드 로직을 안전하게 다듬어서 동일한 컨셉으로 목표 각도(rad)를 계산하고
        동시에 장애물 존재 여부를 반환한다.
        - obstacle: 0<range<obstacle_dist 인 인덱스들
        - right_space: 첫 장애물까지
        - left_space : 마지막 장애물부터 끝까지
        - center_space: 장애물-장애물 사이 gap 중 큰 것
        """
        ranges = scan.ranges
        n = len(ranges)

        # 1) 장애물 인덱스 수집
        obstacle = []
        for idx, r in enumerate(ranges):
            if r is None or math.isnan(r) or math.isinf(r):
                continue
            if 0.0 < r < self.obstacle_dist:
                obstacle.append(idx)

        # 2) 장애물이 없으면 직진(각도 0)
        if len(obstacle) == 0:
            return 0.0, False

        # 3) 중앙 갭 탐색 (가장 큰 gap의 중앙 인덱스)
        center_space = 0
        center_index = None
        if len(obstacle) >= 2:
            for i in range(1, len(obstacle)):
                gap = obstacle[i] - obstacle[i - 1]
                if gap > self.between_obstacle and gap > center_space:
                    center_space = gap
                    center_index = (obstacle[i] + obstacle[i - 1]) // 2

        # 4) 좌/우/중앙 중 가장 큰 공간 선택
        right_space = obstacle[0]               # 0 ~ 첫 장애물
        left_space  = (n - 1) - obstacle[-1]    # 마지막 장애물 ~ 끝

        selected = max(left_space, center_space, right_space)

        # 목표 인덱스 선택 (tie는 순서상 left/right/center로 떨어질 수 있음)
        if selected == left_space:
            target_index = (obstacle[-1] + (n - 1)) // 2
        elif selected == right_space:
            target_index = obstacle[0] // 2
        else:
            # center 선택인데 center_index가 None이면 중앙으로 fallback
            target_index = center_index if center_index is not None else (n // 2)

        # 5) 인덱스를 각도(rad)로 변환
        target_angle_rad = scan.angle_min + scan.angle_increment * target_index

        return target_angle_rad, True


def main():
    _ = LimoSoloPerception()
    rospy.spin()


if __name__ == "__main__":
    main()
