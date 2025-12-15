#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LimoObstacleDecision:
    """
    [DECISION 역할]
    - /limo_solo/target_angle_rad(Float32) 구독: "어느 방향으로 들어가야 하는지(목표 각도)"
    - /scan(LaserScan) 구독: "전방이 얼마나 막혔는지(front_min)"
    - front_min이 가까우면 전진 금지하고 제자리 회전으로 먼저 정렬
    - 조금 열리면 creep_speed로 천천히 진입
    - 충분히 열리면 forward_speed로 전진
    - target_angle 입력이 timeout이면 정지
    """

    def __init__(self):
        # === Pub ===
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # === Sub ===
        self.in_topic   = rospy.get_param("~in_topic", "/limo_solo/target_angle_rad")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")

        rospy.Subscriber(self.in_topic, Float32, self.cb_target, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

        # === Control params ===
        self.forward_speed = rospy.get_param("~forward_speed", 0.10)
        self.creep_speed   = rospy.get_param("~creep_speed", 0.05)

        self.angular_gain  = rospy.get_param("~angular_gain", 1.00)
        self.max_ang_z     = rospy.get_param("~max_ang_z", 1.2)

        # 전방 거리 기반 게이트
        self.front_cone_deg = rospy.get_param("~front_cone_deg", 12.0)   # front_min 측정 각도
        self.hard_stop_dist = rospy.get_param("~hard_stop_dist", 0.15)   # 너무 가까우면 무조건 정지
        self.stop_dist      = rospy.get_param("~stop_dist", 0.30)        # 이하면 전진 0 + 회전만
        self.go_dist        = rospy.get_param("~go_dist", 0.55)          # 이 이상이면 정상 전진

        # timeout
        self.timeout_sec = rospy.get_param("~timeout_sec", 0.50)

        # === State ===
        self.last_target_angle = 0.0
        self.last_msg_time = rospy.Time(0)

        self.front_min = 999.0
        self.last_scan_time = rospy.Time(0)

        rospy.loginfo("[obstacle] init done (sub=%s + %s -> pub=/cmd_vel)", self.in_topic, self.scan_topic)

    # ---------------- Callbacks ----------------
    def cb_target(self, msg: Float32):
        self.last_target_angle = float(msg.data)
        self.last_msg_time = rospy.Time.now()

    def cb_scan(self, scan: LaserScan):
        self.front_min = self.compute_front_min(scan, self.front_cone_deg)
        self.last_scan_time = rospy.Time.now()

    def shutdown_hook(self):
        rospy.loginfo("[obstacle] shutdown... stop robot")
        self.cmd_pub.publish(Twist())

    # ---------------- Step ----------------
    def mission_obstacle_step(self):
        cmd = Twist()

        now = rospy.Time.now()
        dt = (now - self.last_msg_time).to_sec() if self.last_msg_time != rospy.Time(0) else 1e9

        # 1) target 입력이 끊기면 안전 정지
        if dt > self.timeout_sec:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(1.0, f"[obstacle] target timeout ({dt:.2f}s) -> STOP")
            self.cmd_pub.publish(cmd)
            return

        # 2) 조향(각도 추종)
        ang = self.angular_gain * self.last_target_angle
        ang = max(-self.max_ang_z, min(self.max_ang_z, ang))

        # 3) 전방 거리 기반 속도 결정
        fm = float(self.front_min)

        if fm < self.hard_stop_dist:
            # 너무 가까움: 완전 정지 (필요하면 여기서 후진 로직 추가 가능)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(1.0, f"[obstacle] HARD STOP front_min={fm:.2f}m")
        elif fm < self.stop_dist:
            # 전방이 막힘: 전진 금지, 제자리 회전으로 gap 방향 정렬
            cmd.linear.x = 0.0
            cmd.angular.z = ang if abs(ang) > 0.05 else (0.6 if ang >= 0 else -0.6)
        elif fm < self.go_dist:
            # 조금 열림: creep 전진 + 회전
            # (선형 스케일로 속도 천천히 증가)
            t = (fm - self.stop_dist) / max(1e-6, (self.go_dist - self.stop_dist))
            v = self.creep_speed + t * (self.forward_speed - self.creep_speed)
            cmd.linear.x = float(max(0.0, min(self.forward_speed, v)))
            cmd.angular.z = ang
        else:
            # 충분히 열림: 정상 전진
            cmd.linear.x = self.forward_speed
            cmd.angular.z = ang

        self.cmd_pub.publish(cmd)

    # ---------------- Utils ----------------
    @staticmethod
    def compute_front_min(scan: LaserScan, cone_deg: float) -> float:
        ranges = np.array(scan.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return 999.0

        rmax = float(scan.range_max) if scan.range_max > 0 else 10.0
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(np.isinf(ranges), rmax, ranges)
        ranges = np.where(ranges <= 0.0, 0.0, ranges)

        cone = math.radians(cone_deg)
        # 전방(0rad) 주변 [-cone, +cone] 인덱스
        i0 = int(round(((-cone) - scan.angle_min) / scan.angle_increment)) if scan.angle_increment != 0 else 0
        i1 = int(round(((+cone) - scan.angle_min) / scan.angle_increment)) if scan.angle_increment != 0 else n - 1
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))
        if i0 > i1:
            i0, i1 = i1, i0

        fr = ranges[i0:i1 + 1]
        fr = fr[fr > 0.0]  # 0 제거
        if fr.size == 0:
            return 999.0
        return float(np.min(fr))
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LimoObstacleDecision:
    """
    [DECISION 역할]
    - /limo_solo/target_angle_rad(Float32) 구독: "어느 방향으로 들어가야 하는지(목표 각도)"
    - /scan(LaserScan) 구독: "전방이 얼마나 막혔는지(front_min)"
    - front_min이 가까우면 전진 금지하고 제자리 회전으로 먼저 정렬
    - 조금 열리면 creep_speed로 천천히 진입
    - 충분히 열리면 forward_speed로 전진
    - target_angle 입력이 timeout이면 정지
    """

    def __init__(self):
        # === Pub ===
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # === Sub ===
        self.in_topic   = rospy.get_param("~in_topic", "/limo_solo/target_angle_rad")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")

        rospy.Subscriber(self.in_topic, Float32, self.cb_target, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

        # === Control params ===
        self.forward_speed = rospy.get_param("~forward_speed", 0.10)
        self.creep_speed   = rospy.get_param("~creep_speed", 0.05)

        self.angular_gain  = rospy.get_param("~angular_gain", 1.00)
        self.max_ang_z     = rospy.get_param("~max_ang_z", 1.2)

        # 전방 거리 기반 게이트
        self.front_cone_deg = rospy.get_param("~front_cone_deg", 12.0)   # front_min 측정 각도
        self.hard_stop_dist = rospy.get_param("~hard_stop_dist", 0.15)   # 너무 가까우면 무조건 정지
        self.stop_dist      = rospy.get_param("~stop_dist", 0.30)        # 이하면 전진 0 + 회전만
        self.go_dist        = rospy.get_param("~go_dist", 0.55)          # 이 이상이면 정상 전진

        # timeout
        self.timeout_sec = rospy.get_param("~timeout_sec", 0.50)

        # === State ===
        self.last_target_angle = 0.0
        self.last_msg_time = rospy.Time(0)

        self.front_min = 999.0
        self.last_scan_time = rospy.Time(0)

        rospy.loginfo("[obstacle] init done (sub=%s + %s -> pub=/cmd_vel)", self.in_topic, self.scan_topic)

    # ---------------- Callbacks ----------------
    def cb_target(self, msg: Float32):
        self.last_target_angle = float(msg.data)
        self.last_msg_time = rospy.Time.now()

    def cb_scan(self, scan: LaserScan):
        self.front_min = self.compute_front_min(scan, self.front_cone_deg)
        self.last_scan_time = rospy.Time.now()

    def shutdown_hook(self):
        rospy.loginfo("[obstacle] shutdown... stop robot")
        self.cmd_pub.publish(Twist())

    # ---------------- Step ----------------
    def mission_obstacle_step(self):
        cmd = Twist()

        now = rospy.Time.now()
        dt = (now - self.last_msg_time).to_sec() if self.last_msg_time != rospy.Time(0) else 1e9

        # 1) target 입력이 끊기면 안전 정지
        if dt > self.timeout_sec:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(1.0, f"[obstacle] target timeout ({dt:.2f}s) -> STOP")
            self.cmd_pub.publish(cmd)
            return

        # 2) 조향(각도 추종)
        ang = self.angular_gain * self.last_target_angle
        ang = max(-self.max_ang_z, min(self.max_ang_z, ang))

        # 3) 전방 거리 기반 속도 결정
        fm = float(self.front_min)

        if fm < self.hard_stop_dist:
            # 너무 가까움: 완전 정지 (필요하면 여기서 후진 로직 추가 가능)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn_throttle(1.0, f"[obstacle] HARD STOP front_min={fm:.2f}m")
        elif fm < self.stop_dist:
            # 전방이 막힘: 전진 금지, 제자리 회전으로 gap 방향 정렬
            cmd.linear.x = 0.0
            cmd.angular.z = ang if abs(ang) > 0.05 else (0.6 if ang >= 0 else -0.6)
        elif fm < self.go_dist:
            # 조금 열림: creep 전진 + 회전
            # (선형 스케일로 속도 천천히 증가)
            t = (fm - self.stop_dist) / max(1e-6, (self.go_dist - self.stop_dist))
            v = self.creep_speed + t * (self.forward_speed - self.creep_speed)
            cmd.linear.x = float(max(0.0, min(self.forward_speed, v)))
            cmd.angular.z = ang
        else:
            # 충분히 열림: 정상 전진
            cmd.linear.x = self.forward_speed
            cmd.angular.z = ang

        self.cmd_pub.publish(cmd)

    # ---------------- Utils ----------------
    @staticmethod
    def compute_front_min(scan: LaserScan, cone_deg: float) -> float:
        ranges = np.array(scan.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return 999.0

        rmax = float(scan.range_max) if scan.range_max > 0 else 10.0
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(np.isinf(ranges), rmax, ranges)
        ranges = np.where(ranges <= 0.0, 0.0, ranges)

        cone = math.radians(cone_deg)
        # 전방(0rad) 주변 [-cone, +cone] 인덱스
        i0 = int(round(((-cone) - scan.angle_min) / scan.angle_increment)) if scan.angle_increment != 0 else 0
        i1 = int(round(((+cone) - scan.angle_min) / scan.angle_increment)) if scan.angle_increment != 0 else n - 1
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))
        if i0 > i1:
            i0, i1 = i1, i0

        fr = ranges[i0:i1 + 1]
        fr = fr[fr > 0.0]  # 0 제거
        if fr.size == 0:
            return 999.0
        return float(np.min(fr))
