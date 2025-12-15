#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Pub:
    """
    /scan(LaserScan)을 구독해서,
    0.5m 이내 장애물 분포를 보고 가장 '빈 공간'이 큰 방향으로 조향하면서
    /cmd_vel(Twist)을 퍼블리시하는 간단한 갭(gap) 기반 회피 예제.
    """

    def __init__(self):
        # 1) ROS 노드 초기화
        rospy.init_node("Pub_node")

        # 2) Publisher / Subscriber 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)

        # 3) 주기 설정(10Hz)
        self.rate = rospy.Rate(10)

        # 4) 메시지/플래그 초기화
        self.cmd_msg = Twist()
        self.lidar_msg = None        # 아직 scan이 안 들어왔으면 None
        self.lidar_flag = False      # scan을 1번이라도 받았는지

        # 5) 각도(degree) 테이블(인덱스 -> degree) 저장용
        self.degrees = []
        self.degree_flag = False     # degrees를 한 번만 계산하려는 플래그

        # 6) 파라미터(원 코드 값 유지)
        self.between_obstacle = 10   # 장애물 인덱스 간격이 10 이상이면 "중앙 통로"로 인식
        self.obstacle_dist = 0.5     # 0~0.5m는 장애물로 판단

        self.forward_speed = 0.1     # 전진 속도
        self.angular_gain = 0.01     # degree -> angular.z로 바꾸는 단순 게인(원 코드: *0.01)

    def func(self):
        """
        메인 루프에서 계속 호출.
        최신 LiDAR 데이터를 기반으로 cmd_vel 생성 후 publish.
        """
        if not self.lidar_flag or self.lidar_msg is None:
            # callback이 아직 한 번도 안 온 상태
            rospy.logwarn_throttle(1.0, "cb_data 없음 (/scan 미수신)")
            self.rate.sleep()
            return

        # -------- 1) degrees 테이블 초기 1회 생성 --------
        # LaserScan의 index -> angle(deg) 변환 테이블을 한 번 만들어 둠
        # (대부분 scan 배열 길이는 고정이지만, 변할 수도 있어 길이가 바뀌면 재생성)
        n = len(self.lidar_msg.ranges)
        if (not self.degree_flag) or (len(self.degrees) != n):
            self.degrees = []
            for i in range(n):
                angle_rad = self.lidar_msg.angle_min + self.lidar_msg.angle_increment * i
                self.degrees.append(math.degrees(angle_rad))
            self.degree_flag = True

        # -------- 2) 기본 전진 속도 설정 --------
        self.cmd_msg.linear.x = self.forward_speed
        self.cmd_msg.linear.y = 0.0
        self.cmd_msg.linear.z = 0.0
        self.cmd_msg.angular.x = 0.0
        self.cmd_msg.angular.y = 0.0

        # -------- 3) 장애물 인덱스 수집 --------
        # 0 < range < obstacle_dist(0.5m) 인 곳을 "장애물"로 판단
        obstacle = []
        for idx, r in enumerate(self.lidar_msg.ranges):
            # LaserScan에는 inf/nan 등이 섞일 수 있음 -> 방어적으로 처리
            if r is None or math.isnan(r) or math.isinf(r):
                continue

            if 0.0 < r < self.obstacle_dist:
                obstacle.append(idx)

        # -------- 4) 빈 공간(left/right/center) 계산 --------
        # center_space는 "장애물-장애물 사이 간격이 큰 곳"을 통로로 본 것
        center_space = 0
        center_index = None  # (주의) 원 코드에서 center_index 미정의 가능성이 있어 안전하게 None 처리

        # 장애물이 2개 이상일 때만 중앙 갭 탐색
        if len(obstacle) >= 2:
            # 연속된 장애물 인덱스들 사이의 gap을 보며,
            # between_obstacle 이상 벌어진 곳이 있으면 통로로 판단
            for i in range(1, len(obstacle)):
                gap = obstacle[i] - obstacle[i - 1]
                if gap > self.between_obstacle and gap > center_space:
                    center_space = gap
                    center_index = (obstacle[i] + obstacle[i - 1]) // 2

        # -------- 5) 어느 방향으로 갈지 선택 --------
        # 장애물이 하나라도 있으면:
        # - right_space: 배열 시작~첫 장애물까지
        # - left_space : 마지막 장애물~배열 끝까지
        # - center_space: 장애물 사이 큰 gap
        if len(obstacle) > 0:
            right_space = obstacle[0]
            left_space = (n - 1) - obstacle[-1]  # 끝 인덱스 기준으로 계산

            selected_space = max(left_space, center_space, right_space)

            # 선택된 공간의 "중앙 인덱스"를 목표로 잡기
            if selected_space == left_space:
                # 왼쪽 끝쪽이 가장 넓음
                rospy.loginfo_throttle(0.5, "left")
                target_index = (obstacle[-1] + (n - 1)) // 2

            elif selected_space == right_space:
                # 오른쪽 끝쪽이 가장 넓음
                rospy.loginfo_throttle(0.5, "right")
                target_index = obstacle[0] // 2

            else:
                # 중앙 통로가 가장 넓음
                rospy.loginfo_throttle(0.5, "center")
                # center_index가 None일 수 있으므로 방어 처리
                if center_index is None:
                    target_index = n // 2
                else:
                    target_index = center_index

            # 목표 인덱스에 해당하는 각도(deg)
            target_deg = self.degrees[target_index]

        else:
            # 장애물이 없으면 직진(각도 0)
            target_deg = 0.0

        # -------- 6) 최종 조향값 생성 --------
        # degree 값을 단순히 게인 곱해서 angular.z로 사용(원 코드 유지)
        self.cmd_msg.angular.z = target_deg * self.angular_gain

        # (선택) 너무 큰 회전값 방지하고 싶다면 아래처럼 포화(saturation) 가능
        # self.cmd_msg.angular.z = max(-1.5, min(1.5, self.cmd_msg.angular.z))

        rospy.loginfo_throttle(0.2, f"cmd angular.z = {self.cmd_msg.angular.z:.3f}")

        # -------- 7) publish --------
        self.pub.publish(self.cmd_msg)
        self.rate.sleep()

    def callback(self, msg: LaserScan):
        """ /scan 콜백: 최신 LiDAR 메시지 저장 """
        self.lidar_flag = True
        self.lidar_msg = msg


def main():
    pub = Pub()
    while not rospy.is_shutdown():
        pub.func()


if __name__ == "__main__":
    main()
