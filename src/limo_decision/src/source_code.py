#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from dataclasses import dataclass

from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from laser_geometry import LaserProjection


@dataclass
class Point2D:
    x: float
    y: float


class ClusterFollower(object):
    def __init__(self):
        # --- ROS pub/sub 설정 ---
        self.sub_scan = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback, queue_size=1
        )

        self.marker_pub = rospy.Publisher(
            "/dbscan_lines", Marker, queue_size=300
        )
        self.pub_cloud = rospy.Publisher(
            "/scan_points", PointCloud2, queue_size=300
        )
        self.pub_speed = rospy.Publisher(
            "/commands/motor/speed", Float64, queue_size=1
        )
        self.pub_servo = rospy.Publisher(
            "/commands/servo/position", Float64, queue_size=1
        )

        self.projector = LaserProjection()

        # --- 파라미터 (C++ 코드와 동일) ---
        self.eps = 0.2           # 클러스터 간 거리 (20~25cm)
        self.min_samples = 2     # 최소 점 수
        self.follow_speed = 1100 # 주행 속도
        self.min_speed = 900     # 인식 안 될 때 속도
        self.k_yaw = 1.2         # 조향 비율

    # ------------------ DBSCAN ------------------
    def dbscan(self, points):
        n = len(points)
        labels = [-1] * n
        cluster_id = 0

        def dist(a, b):
            return math.hypot(a.x - b.x, a.y - b.y)

        for i in range(n):
            if labels[i] != -1:
                continue

            neighbors = []
            for j in range(n):
                if dist(points[i], points[j]) <= self.eps:
                    neighbors.append(j)

            if len(neighbors) < self.min_samples:
                # noise (그대로 -1 유지)
                continue

            cluster_id += 1
            labels[i] = cluster_id

            seed_set = list(neighbors)
            k = 0
            while k < len(seed_set):
                j = seed_set[k]
                k += 1

                if labels[j] != -1:
                    continue

                labels[j] = cluster_id

                j_neighbors = []
                for m in range(n):
                    if dist(points[j], points[m]) <= self.eps:
                        j_neighbors.append(m)

                if len(j_neighbors) >= self.min_samples:
                    seed_set.extend(j_neighbors)

        return labels

    # ------------------ LaserScan 콜백 ------------------
    def scan_callback(self, scan):
        points = []
        angle = scan.angle_min

        # --- 포인트 필터링: 13cm ~ 90cm ---
        for r in scan.ranges:
            if (not math.isfinite(r)) or r < 0.13 or r > 0.9:
                angle += scan.angle_increment
                continue

            # 전방 180도 기준 ±120도 제한
            angle_deg = angle * 180.0 / math.pi
            if angle_deg < 0.0:
                angle_deg += 360.0

            # 60~300 도 사이만 사용 (전방-기준)
            if angle_deg < 60.0 or angle_deg > 300.0:
                angle += scan.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append(Point2D(x, y))

            angle += scan.angle_increment

        if not points:
            rospy.logwarn_throttle(1.0, "No valid points detected — moving straight slowly")
            self.publish_minimal_speed(move_forward=True)
            return

        # --- PointCloud2 퍼블리시 (원본 scan 기준) ---
        cloud = PointCloud2()
        cloud = self.projector.projectLaser(scan)
        self.pub_cloud.publish(cloud)

        # --- DBSCAN ---
        labels = self.dbscan(points)
        if labels:
            max_label = max(labels)
        else:
            max_label = -1

        rospy.loginfo_throttle(1.0, "Clusters detected: %d", max_label)

        left_x_sum = 0.0
        left_y_sum = 0.0
        right_x_sum = 0.0
        right_y_sum = 0.0
        left_cnt = 0
        right_cnt = 0

        # --- 클러스터 중심 계산 + Marker ---
        for c in range(1, max_label + 1):
            cx = 0.0
            cy = 0.0
            count = 0
            for i, p in enumerate(points):
                if labels[i] == c:
                    cx += p.x
                    cy += p.y
                    count += 1

            if count > 0:
                cx /= count
                cy /= count

                if cy > 0:
                    left_cnt += 1
                    left_x_sum += cx
                    left_y_sum += cy
                else:
                    right_cnt += 1
                    right_x_sum += cx
                    right_y_sum += cy

                marker = Marker()
                marker.header.frame_id = scan.header.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "cluster"
                marker.id = c
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = cx
                marker.pose.position.y = cy
                marker.pose.position.z = 0.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime = rospy.Duration(0.1)
                self.marker_pub.publish(marker)

        # --- 목표 좌표 계산 ---
        target_x = 0.0
        target_y = 0.0
        cmd_speed = self.follow_speed

        if left_cnt > 0 and right_cnt > 0:
            left_x = left_x_sum / left_cnt
            left_y = left_y_sum / left_cnt
            right_x = right_x_sum / right_cnt
            right_y = right_y_sum / right_cnt

            total_cnt = left_cnt + right_cnt
            weight_left = float(right_cnt) / total_cnt
            weight_right = float(left_cnt) / total_cnt

            target_x = weight_left * left_x + weight_right * right_x
            target_y = weight_left * left_y + weight_right * right_y

        elif left_cnt > 0:
            target_x = left_x_sum / left_cnt
            # 왼쪽 → 오른쪽 피하기
            target_y = (left_y_sum / left_cnt) - 0.12

        elif right_cnt > 0:
            target_x = right_x_sum / right_cnt
            # 오른쪽 → 왼쪽 피하기
            target_y = (right_y_sum / right_cnt) + 0.12

        else:
            rospy.logwarn_throttle(1.0, "No clusters detected — moving straight slowly")
            self.publish_minimal_speed(move_forward=True)
            return

        yaw = math.atan2(target_y, target_x)
        yaw *= self.k_yaw

        # yaw → 0~1 servo 스케일 변환
        yaw_deg = yaw * 180.0 / math.pi

        if yaw_deg < 0.0:
            # 왼쪽을 더 급격하게 회전
            steering = 0.545 + (yaw_deg / 25.0) * 0.545
        else:
            steering = 0.545 + (yaw_deg / 20.0) * (1.0 - 0.545)

        # 0.0 ~ 1.0 사이로 클램프
        steering = max(0.0, min(1.0, steering))

        speed_msg = Float64()
        servo_msg = Float64()
        speed_msg.data = cmd_speed
        servo_msg.data = steering

        self.pub_speed.publish(speed_msg)
        self.pub_servo.publish(servo_msg)

        rospy.loginfo_throttle(1.0, "Speed=%.2f, Steering=%.2f", cmd_speed, steering)

    # ------------------ 최소 속도로 전진/정지 ------------------
    def publish_minimal_speed(self, move_forward=True):
        speed_msg = Float64()
        servo_msg = Float64()

        if move_forward:
            speed_msg.data = self.min_speed   # 천천히 전진
            servo_msg.data = 0.545            # 직진 (원래 값 0.57165)
            rospy.loginfo_throttle(1.0, "→ No target, moving forward slowly")
        else:
            speed_msg.data = 0.0              # 완전 정지
            servo_msg.data = 0.545
            rospy.loginfo_throttle(1.0, "→ Stopping due to no target")

        self.pub_speed.publish(speed_msg)
        self.pub_servo.publish(servo_msg)


if __name__ == "__main__":
    rospy.init_node("cluster_follower")
    node = ClusterFollower()
    rospy.spin()
