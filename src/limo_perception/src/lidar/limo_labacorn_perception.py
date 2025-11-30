#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Bool
from laser_geometry import LaserProjection

# ========== 전역 퍼블리셔 ==========
pub_yaw = None         # Float32, rad
pub_valid = None       # Bool, 타겟 유효 여부
marker_pub = None      # 클러스터 중심 Marker
pub_cloud = None       # PointCloud2 (디버그용)
projector = None

# ========== 파라미터 (launch에서 override 가능) ==========
eps = 0.1              # DBSCAN 이웃 거리 [m]
min_samples = 2        # 최소 이웃 점 개수

range_min = 0.13       # 라이다 사용 최소 거리 [m]
range_max = 0.80       # 라이다 사용 최대 거리 [m]

# 갭 네비게이션용
fov_deg           = 80.0   # 전방에서 사용할 FOV (±fov_deg)
min_gap_width     = 0.4    # 갭 최소 폭 [m] (바퀴간 0.25m + 여유 0.1m)
target_dist_factor = 0.7   # 갭 안쪽으로 들어갈 거리 비율 (range_max * factor)


# ========== DBSCAN 구현 (튜플만 사용) ==========
def dbscan(points):
    """
    points: [(x, y), ...]
    return: labels (길이 N 리스트), -1 = noise, 1..K = cluster id
    """
    n = len(points)
    if n == 0:
        return []

    labels = [-1] * n
    cluster_id = 0

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    for i in range(n):
        if labels[i] != -1:
            continue

        neighbors = []
        for j in range(n):
            if dist(points[i], points[j]) <= eps:
                neighbors.append(j)

        if len(neighbors) < min_samples:
            continue

        cluster_id += 1
        labels[i] = cluster_id
        seed_set = list(neighbors)

        k = 0
        while k < len(seed_set):
            j = seed_set[k]

            if labels[j] == -1:
                labels[j] = cluster_id

                j_neighbors = []
                for m in range(n):
                    if dist(points[j], points[m]) <= eps:
                        j_neighbors.append(m)

                if len(j_neighbors) >= min_samples:
                    seed_set.extend(j_neighbors)

            k += 1

    return labels


def publish_no_target():
    """타겟이 없을 때: yaw=0, valid=False 퍼블리시"""
    yaw_msg = Float32()
    yaw_msg.data = 0.0
    valid_msg = Bool()
    valid_msg.data = False
    pub_yaw.publish(yaw_msg)
    pub_valid.publish(valid_msg)


# ========== LaserScan 콜백 (Perception 핵심) ==========
def scan_callback(scan):
    global pub_yaw, pub_valid, marker_pub, pub_cloud, projector
    global range_min, range_max, fov_deg, min_gap_width, target_dist_factor

    points = []
    angle = scan.angle_min

    # 1) range 필터링 + (x, y) 포인트 생성
    for r in scan.ranges:
        if (not math.isfinite(r)) or r < range_min or r > range_max:
            angle += scan.angle_increment
            continue

        x = r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((x, y))
        angle += scan.angle_increment

    if not points:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] No valid points detected")
        publish_no_target()
        return

    # 2) PointCloud2 발행 (디버깅용)
    try:
        cloud = projector.projectLaser(scan)
        pub_cloud.publish(cloud)
    except Exception as e:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] LaserProjection failed: %s", str(e))

    # 3) DBSCAN 클러스터링
    labels = dbscan(points)
    if not labels:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] DBSCAN returned no labels")
        publish_no_target()
        return

    max_label = max(labels)
    if max_label <= 0:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] No clusters detected (all noise)")
        publish_no_target()
        return

    rospy.loginfo_throttle(1.0, "[labacorn_perception] Clusters detected: %d", max_label)

    # 4) 각 클러스터 중심 계산 + Marker
    centers = []   # [(cx, cy), ...]
    for c in range(1, max_label + 1):
        cx = 0.0
        cy = 0.0
        count = 0

        for i, p in enumerate(points):
            if labels[i] == c:
                cx += p[0]
                cy += p[1]
                count += 1

        if count == 0:
            continue

        cx /= float(count)
        cy /= float(count)
        centers.append((cx, cy))

        # RViz Marker
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
        marker_pub.publish(marker)

    if not centers:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] No cluster centers")
        publish_no_target()
        return

    # 5) 갭 네비게이션: 전방 FOV 안에서 가장 넓은 틈 찾기
    fov_rad = math.radians(fov_deg)

    # 5-1) 극좌표(θ, r)로 변환 + 전방 FOV 필터
    obstacles = []   # [(theta, r), ...]
    for (cx, cy) in centers:
        r = math.hypot(cx, cy)
        if r < range_min or r > range_max:
            continue

        theta = math.atan2(cy, cx)  # -pi ~ +pi, 0 = 정면

        if abs(theta) > fov_rad:
            continue

        obstacles.append((theta, r))

    if not obstacles:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] No obstacles in FOV")
        publish_no_target()
        return

    # 5-2) 각도 기준 정렬
    obstacles.sort(key=lambda x: x[0])

    # 5-3) FOV 경계에 가상 장애물 추가
    boundary = [(-fov_rad, None)] + obstacles + [(+fov_rad, None)]

    best_gap = None
    best_score = -1.0

    for i in range(len(boundary) - 1):
        th1, r1 = boundary[i]
        th2, r2 = boundary[i + 1]

        gap_theta = th2 - th1  # 라디안
        if gap_theta <= 0.0:
            continue

        # 두 장애물(또는 경계) 중 가까운 거리 사용, 없으면 range_max
        d = range_max
        if r1 is not None and r2 is not None:
            d = min(r1, r2)
        elif r1 is not None:
            d = r1
        elif r2 is not None:
            d = r2

        gap_width = d * gap_theta   # 대략적인 갭 폭 [m]

        # 로봇 폭 + 여유보다 좁으면 패스
        if gap_width < min_gap_width:
            continue

        # 갭 중심각 계산
        th_center = 0.5 * (th1 + th2)

        # score: 폭이 클수록, 정면(θ=0)에 가까울수록 좋게
        center_penalty = 1.0 - (abs(th_center) / fov_rad)  # 0~1
        score = gap_width * (0.5 + 0.5 * center_penalty)

        if score > best_score:
            best_score = score
            best_gap = (th1, th2, d)

    if best_gap is None:
        rospy.logwarn_throttle(1.0, "[labacorn_perception] No wide enough gap")
        publish_no_target()
        return

    th1, th2, d = best_gap
    gap_center = 0.5 * (th1 + th2)

    # 5-4) 갭 안쪽의 목표 거리 설정
    if d is None:
        d = range_max
    r_target = min(d, target_dist_factor * range_max)
    if r_target < range_min:
        r_target = range_min + 0.05

    target_x = r_target * math.cos(gap_center)
    target_y = r_target * math.sin(gap_center)

    # 6) yaw 계산 (decision에서 gain, saturation 적용 예정)
    yaw = math.atan2(target_y, target_x)  # gap_center와 거의 동일

    yaw_msg = Float32()
    yaw_msg.data = yaw

    valid_msg = Bool()
    valid_msg.data = True

    pub_yaw.publish(yaw_msg)
    pub_valid.publish(valid_msg)

    yaw_deg = yaw * 180.0 / math.pi
    rospy.loginfo_throttle(
        1.0,
        "[labacorn_perception] target yaw=%.3f rad (%.1f deg), gap_score=%.3f",
        yaw, yaw_deg, best_score
    )


# ========== main ==========
if __name__ == "__main__":
    rospy.init_node("limo_labacorn_perception")

    # 토픽 이름 파라미터
    scan_topic   = rospy.get_param("~scan_topic", "/scan")
    marker_topic = rospy.get_param("~marker_topic", "/labacorn/dbscan_centers")
    cloud_topic  = rospy.get_param("~cloud_topic", "/labacorn/scan_points")
    yaw_topic    = rospy.get_param("~yaw_topic", "/labacorn/yaw")
    valid_topic  = rospy.get_param("~valid_topic", "/labacorn/valid")

    # 퍼블리셔
    pub_yaw    = rospy.Publisher(yaw_topic,   Float32,      queue_size=1)
    pub_valid  = rospy.Publisher(valid_topic, Bool,         queue_size=1)
    marker_pub = rospy.Publisher(marker_topic, Marker,      queue_size=300)
    pub_cloud  = rospy.Publisher(cloud_topic, PointCloud2,  queue_size=300)
    projector  = LaserProjection()

    # 파라미터 로드
    eps                = rospy.get_param("~eps",                eps)
    min_samples        = rospy.get_param("~min_samples",        min_samples)
    range_min          = rospy.get_param("~range_min",          range_min)
    range_max          = rospy.get_param("~range_max",          range_max)
    fov_deg            = rospy.get_param("~fov_deg",            fov_deg)
    min_gap_width      = rospy.get_param("~min_gap_width",      min_gap_width)
    target_dist_factor = rospy.get_param("~target_dist_factor", target_dist_factor)

    rospy.Subscriber(scan_topic, LaserScan, scan_callback, queue_size=1)

    rospy.loginfo("Limo Labacorn Perception node started.")
    rospy.spin()
