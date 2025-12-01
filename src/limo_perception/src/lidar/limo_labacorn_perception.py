#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass

import rospy
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker


@dataclass
class Point2D:
    x: float
    y: float


class LabacornPerceptionNode:
    """
    LiDAR 기반 Labacorn(갭) 인지 노드.
    - /scan 을 입력으로 받아 DBSCAN 기반으로 장애물을 그룹화
    - 좌우 클러스터를 이용해 목표 지점을 계산하고 yaw(rad) 를 Float32 로 퍼블리시
    - 감지 여부는 /labacorn_detected (Bool) 로 퍼블리시
    """

    def __init__(self):
        # ----- 파라미터 -----
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.marker_topic = rospy.get_param("~marker_topic", "/cluster_info")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/point_cloud")
        self.target_topic = rospy.get_param("~target_topic", "/labacorn/target")
        self.detected_topic = rospy.get_param("~detected_topic", "/labacorn_detected")

        self.eps = rospy.get_param("~eps", 0.2)  # DBSCAN 이웃 거리
        self.min_samples = rospy.get_param("~min_samples", 2)
        self.range_min = rospy.get_param("~range_min", 0.13)
        self.range_max = rospy.get_param("~range_max", 0.9)
        self.fov_min_deg = rospy.get_param("~fov_min_deg", 60.0)
        self.fov_max_deg = rospy.get_param("~fov_max_deg", 300.0)
        self.single_side_offset = rospy.get_param("~single_side_offset", 0.12)

        # ----- ROS pub/sub -----
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=100)
        self.pub_cloud = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=10)
        self.pub_target = rospy.Publisher(self.target_topic, Float32, queue_size=1)
        self.pub_detected = rospy.Publisher(self.detected_topic, Bool, queue_size=1)

        self.projector = LaserProjection()

        self.sub_scan = rospy.Subscriber(
            self.scan_topic, LaserScan, self.scan_callback, queue_size=1
        )

        rospy.loginfo(
            "[labacorn_perception] listening on %s, publishing target -> %s",
            self.scan_topic,
            self.target_topic,
        )

    def publish_detection(self, yaw: float, detected: bool):
        target_msg = Float32()
        target_msg.data = yaw
        detected_msg = Bool()
        detected_msg.data = detected

        self.pub_target.publish(target_msg)
        self.pub_detected.publish(detected_msg)

    def dbscan(self, points):
        n = len(points)
        labels = [-1] * n
        cluster_id = 0

        def dist(a: Point2D, b: Point2D):
            return math.hypot(a.x - b.x, a.y - b.y)

        for i in range(n):
            if labels[i] != -1:
                continue

            neighbors = []
            for j in range(n):
                if dist(points[i], points[j]) <= self.eps:
                    neighbors.append(j)

            if len(neighbors) < self.min_samples:
                continue

            cluster_id += 1
            labels[i] = cluster_id
            seed = list(neighbors)
            k = 0

            while k < len(seed):
                j = seed[k]
                k += 1

                if labels[j] != -1:
                    continue

                labels[j] = cluster_id
                j_neighbors = []
                for m in range(n):
                    if dist(points[j], points[m]) <= self.eps:
                        j_neighbors.append(m)

                if len(j_neighbors) >= self.min_samples:
                    seed.extend(j_neighbors)

        return labels

    def scan_callback(self, scan: LaserScan):
        points = []
        angle = scan.angle_min

        for distance in scan.ranges:
            if (not math.isfinite(distance)) or distance < self.range_min or distance > self.range_max:
                angle += scan.angle_increment
                continue

            angle_deg = math.degrees(angle)
            if angle_deg < 0.0:
                angle_deg += 360.0

            if angle_deg < self.fov_min_deg or angle_deg > self.fov_max_deg:
                angle += scan.angle_increment
                continue

            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append(Point2D(x, y))
            angle += scan.angle_increment

        if not points:
            rospy.logwarn_throttle(1.0, "[labacorn_perception] no valid LiDAR points")
            self.publish_detection(0.0, False)
            return

        try:
            cloud = self.projector.projectLaser(scan)
            self.pub_cloud.publish(cloud)
        except Exception as exc:  # pragma: no cover
            rospy.logwarn_throttle(
                1.0, "[labacorn_perception] LaserProjection failed: %s", str(exc)
            )

        labels = self.dbscan(points)
        if not labels:
            rospy.logwarn_throttle(1.0, "[labacorn_perception] DBSCAN returned no clusters")
            self.publish_detection(0.0, False)
            return

        max_label = max(labels)
        if max_label < 1:
            rospy.logwarn_throttle(1.0, "[labacorn_perception] all points treated as noise")
            self.publish_detection(0.0, False)
            return

        rospy.loginfo_throttle(1.0, "[labacorn_perception] clusters=%d", max_label)

        left_x_sum = 0.0
        left_y_sum = 0.0
        right_x_sum = 0.0
        right_y_sum = 0.0
        left_cnt = 0
        right_cnt = 0

        for c in range(1, max_label + 1):
            cx = 0.0
            cy = 0.0
            count = 0
            for idx, pt in enumerate(points):
                if labels[idx] == c:
                    cx += pt.x
                    cy += pt.y
                    count += 1

            if count == 0:
                continue

            cx /= float(count)
            cy /= float(count)

            if cy >= 0.0:
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
            marker.ns = "labacorn_cluster"
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
            marker.lifetime = rospy.Duration(0.2)
            self.marker_pub.publish(marker)

        if left_cnt == 0 and right_cnt == 0:
            rospy.logwarn_throttle(1.0, "[labacorn_perception] no usable clusters")
            self.publish_detection(0.0, False)
            return

        target_x = 0.0
        target_y = 0.0

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
            target_y = (left_y_sum / left_cnt) - self.single_side_offset
        else:
            target_x = right_x_sum / right_cnt
            target_y = (right_y_sum / right_cnt) + self.single_side_offset

        yaw = math.atan2(target_y, target_x)
        self.publish_detection(yaw, True)

        rospy.loginfo_throttle(
            1.0,
            "[labacorn_perception] yaw=%.3f rad (%.1f deg), L=%d R=%d",
            yaw,
            math.degrees(yaw),
            left_cnt,
            right_cnt,
        )


def main():
    rospy.init_node("limo_labacorn_perception")
    LabacornPerceptionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
