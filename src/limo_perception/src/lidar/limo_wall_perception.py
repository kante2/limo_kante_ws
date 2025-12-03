#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LiDAR 기반 벽 따라가기 인지 노드.
원본 제어 로직(limo_wall_follow_mission2_ver2.py)에서 센서 처리 부분만 분리하여,
결과를 여러 토픽으로 퍼블리시한다. Decision 노드는 이 토픽을 구독해 제어만 담당한다.
"""

import math
from typing import List

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


LEFT = "LEFT"
RIGHT = "RIGHT"


class LimoWallPerception:
    def __init__(self):
        rospy.init_node("limo_wall_perception")

        # Parameters
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.condition_topic = rospy.get_param("~condition_topic", "/wall_follow/condition")
        self.min_distance_topic = rospy.get_param("~min_distance_topic", "/wall_follow/min_distance")
        self.side_dist_topic = rospy.get_param("~side_dist_topic", "/wall_follow/side_distances")
        self.turn_angle_topic = rospy.get_param("~turn_angle_topic", "/wall_follow/turn_angle")
        self.valid_topic = rospy.get_param("~valid_topic", "/wall_follow/valid")

        self.scan_dist = rospy.get_param("~scan_dist", 0.5)
        self.offset = rospy.get_param("~offset", 0.2)
        self.default_angle = rospy.get_param("~default_angle", 0.2)

        # Publishers
        self.condition_pub = rospy.Publisher(self.condition_topic, String, queue_size=1)
        self.min_dist_pub = rospy.Publisher(self.min_distance_topic, Float32, queue_size=1)
        self.side_dist_pub = rospy.Publisher(self.side_dist_topic, Float32MultiArray, queue_size=1)
        self.turn_angle_pub = rospy.Publisher(self.turn_angle_topic, Float32, queue_size=1)
        self.valid_pub = rospy.Publisher(self.valid_topic, Bool, queue_size=1)

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo(
            "[wall_perception] listening on %s, publishing condition -> %s",
            self.scan_topic,
            self.condition_topic,
        )

    def scan_callback(self, scan: LaserScan):
        degrees = [
            (scan.angle_min + (i * scan.angle_increment)) * 180.0 / math.pi
            for i in range(len(scan.ranges))
        ]

        front_distances: List[float] = [
            rng
            for idx, rng in enumerate(scan.ranges)
            if (-90.0 <= degrees[idx] < 90.0) and (0.0 < rng <= self.scan_dist)
        ]

        obstacle_indices = [
            idx
            for idx, rng in enumerate(scan.ranges)
            if (-5.0 <= degrees[idx] < 5.0) and (0.05 < rng <= self.scan_dist)
        ]

        min_distance = min(front_distances) if front_distances else 0.0

        # Distances near ±70, ±80 degrees (used for maintain logic)
        left70 = self.angle_distance(scan, degrees, 70.0)
        left80 = self.angle_distance(scan, degrees, 80.0)
        right70 = self.angle_distance(scan, degrees, -70.0)
        right80 = self.angle_distance(scan, degrees, -80.0)

        condition = self.judge_condition(front_distances, obstacle_indices)
        turn_angle = self.compute_turn_angle(scan, obstacle_indices)

        # Publish outputs
        self.condition_pub.publish(String(data=condition))
        self.min_dist_pub.publish(Float32(data=min_distance))

        side_msg = Float32MultiArray()
        side_msg.data = [left70 or 0.0, left80 or 0.0, right70 or 0.0, right80 or 0.0]
        self.side_dist_pub.publish(side_msg)

        self.turn_angle_pub.publish(Float32(data=turn_angle))

        has_wall = bool(front_distances) or bool(obstacle_indices)
        self.valid_pub.publish(Bool(data=has_wall))

    def angle_distance(self, scan: LaserScan, degrees: List[float], degree: float):
        for idx, rng in enumerate(scan.ranges):
            if (degree - 1.0) <= degrees[idx] <= (degree + 1.0) and 0.0 < rng <= 0.6:
                return rng
        return None

    def judge_condition(self, distances: List[float], obstacle_indices: List[int]) -> str:
        if not obstacle_indices:
            return "forward"

        if distances:
            min_distance = min(distances)
            if min_distance < self.scan_dist - self.offset:
                return "close"
            if self.scan_dist - self.offset <= min_distance <= self.scan_dist + self.offset:
                return "maintaining"
            return "far"

        return "obstacle"

    def compute_turn_angle(self, scan: LaserScan, obstacle_indices: List[int]) -> float:
        if not obstacle_indices:
            return 0.0

        obstacle_index = obstacle_indices[-1]
        obstacle_end_point = scan.angle_min + (obstacle_index * scan.angle_increment)
        blank_space = self.default_angle / 2.0
        return obstacle_end_point + blank_space


def main():
    node = LimoWallPerception()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
