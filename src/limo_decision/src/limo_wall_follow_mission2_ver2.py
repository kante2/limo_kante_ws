# 파일 이름: limo_wall_following.py (추정)

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
import time

# 전역 상수 (또는 클래스 외부 정의)
LEFT = "LEFT"
RIGHT = "RIGHT"

class Limo_wall_following:
    # 1. 클래스 초기화 메서드
    def __init__(self, direction):
        # ROS 노드 초기화
        rospy.init_node('laser_scan_node')
        
        # Subscriber: /scan 토픽에서 LaserScan 메시지를 받음
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=1)
        
        # Publisher: /cmd_vel 토픽으로 Twist 메시지(속도/각속도)를 보냄
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.cmd_vel_msg = Twist()
        
        # 상태 변수 초기화
        self.condition = None
        self.speed = 0
        self.angle = 0
        
        self.lidar_flag = False
        self.is_scan = False
        
        self.distance = []
        self.obstacle_data_range = []
        self.obstacle_data_idx = []
        
        self.DIRECTION = direction
        self.default_speed = 0.15
        self.default_angle = 0.2
        self.scan_dist = 0.5
        self.offset = 0.2
        
        self.msg = None
        
    # 2. 레이저 스캔 콜백 메서드
    def laser_callback(self, msg):
        self.msg = msg
        self.lidar_scan()
        self.is_scan = True

    # 3. LiDAR 스캔 데이터 처리 메서드
    def LiDAR_scan(self):
        print("LiDAR_scan")
        self.lidar_flag = False
        
        # 각도 계산을 위한 리스트
        degrees = []
        for i, data in enumerate(self.msg.ranges):
            # i번째 데이터의 각도를 계산 (도 단위)
            degree = (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
            degrees.append(degree)
            
        self.lidar_flag = True
        
        # 원하는 각도 내에 있는 거리만 추출
        self.distance = []
        scan_dist = self.scan_dist # 0.5m
        for i, data in enumerate(self.msg.ranges):
            # 전방 180도 (-90 ~ 90도) 범위 내에서만 처리
            if (-90 <= degrees[i] < 90) and (0 < self.msg.ranges[i] <= scan_dist):
                self.distance.append(self.msg.ranges[i])

        # 전방 디텍팅 각도 (장애물 감지)
        self.obstacle_data_range = []
        self.obstacle_data_idx = []
        for i, data in enumerate(self.msg.ranges):
            # 전방 -5도 ~ 5도 범위 내에서 0.05m ~ scan_dist(0.5m) 사이에 데이터가 있을 경우
            if (-5 <= degrees[i] < 5) and (0.05 < self.msg.ranges[i] <= scan_dist):
                self.obstacle_data_idx.append(i)
                self.obstacle_data_range.append(data)
                
        self.judge_distance()
        
    # 3. 거리 판단 메서드
    def judge_distance(self):
        scan_dist = self.scan_dist
        offset = self.offset # 0.2m
        
        if len(self.obstacle_data_idx) == 0:
            self.condition = "forward"
            self.speed = self.default_speed
            self.angle = 0
        elif len(self.distance) > 0:
            min_distance = min(self.distance)
            
            # (scan_dist - offset) = 0.3m 보다 가까운 경우 (벽/장애물 근접)
            if min_distance < scan_dist - offset:
                self.condition = "close"
            # (scan_dist - offset) = 0.3m ~ (scan_dist + offset) = 0.7m 사이에 있는 경우 (벽 유지)
            elif scan_dist - offset <= min_distance <= scan_dist + offset:
                self.condition = "maintaining"
            # 0.7m 보다 먼 경우 (벽에서 멀어짐)
            else:
                self.condition = "far"
        else: # 장애물 데이터는 있으나, self.distance에 해당하는 거리 데이터가 없는 경우 (전방 180도 외각에 장애물이 있을 수 있음)
            self.condition = "obstacle"

    # 4. 벽 유지 방향 제어 메서드
    def maintain_direction(self):
        if self.DIRECTION == LEFT:
            # -70도와 -80도 방향의 거리 데이터를 가져옴
            angle1 = self.angle_distance(70)
            angle2 = self.angle_distance(80)
            print(f"LEFT: angle1={angle1}, angle2={angle2}")
        elif self.DIRECTION == RIGHT:
            # 70도와 80도 방향의 거리 데이터를 가져옴
            angle1 = self.angle_distance(-70)
            angle2 = self.angle_distance(-80)
            print(f"RIGHT: angle1={angle1}, angle2={angle2}")
            
        if angle2 is not None and angle1 is not None:
            try:
                # angle2, angle1을 이용하여 회전 각도 theta 계산
                theta = acos(angle2 / angle1)
                
                # 라디안을 도(degree)로 변환
                thetad = theta * 180 / pi
                
                # 기본 각도 보정값
                DEFAULT_THETA = 0.28 
                
                print(f"theta: {thetad}")
                
                if 0 <= thetad <= 20:
                    print("각도가 작습니다")
                    # 회전 각도 조정
                    if self.DIRECTION == LEFT:
                        self.angle = theta - DEFAULT_THETA
                    elif self.DIRECTION == RIGHT:
                        self.angle = -(theta - DEFAULT_THETA)
                        
                    self.speed = self.default_speed
                    
                elif thetad > 20:
                    print("각도가 큽니다")
                    # 회전 각도 조정
                    if self.DIRECTION == LEFT:
                        self.angle = -(thetad - DEFAULT_THETA)
                    elif self.DIRECTION == RIGHT:
                        self.angle = theta - DEFAULT_THETA
                        
            except ValueError:
                # acos 계산 중 도메인 오류 발생 시 (e.g. angle2/angle1 > 1)
                self.speed = self.default_speed
                pass
            
        else:
            # angle1 또는 angle2 중 하나라도 데이터를 못 찾은 경우
            self.speed = self.default_speed
            pass

    # 5. 장애물 회피 메서드
    def obstacle_motion(self):
        print("obstacle_motion")
        # LiDAR 데이터의 각도 증가량 (라디안)
        angle_incre = self.msg.angle_increment 
        
        # 장애물 데이터 리스트의 마지막 인덱스
        obstacle_data_idx = self.obstacle_data_idx[-1]
        
        # 장애물 끝점의 각도 (라디안)
        obstacle_end_point = self.msg.angle_min + (obstacle_data_idx * angle_incre)
        
        # 빈 공간 확보를 위한 각도
        blank_space = self.default_angle / 2 
        
        # 회피를 위해 회전해야 할 최종 각도 (라디안)
        turn_angle = obstacle_end_point + blank_space
        
        if self.DIRECTION == LEFT:
            self.angle = turn_angle
        elif self.DIRECTION == RIGHT:
            self.angle = -turn_angle
            
        self.speed = -turn_angle / 10

    # 6. 특정 각도 주변 거리 데이터 가져오기 메서드
    def angle_distance(self, degree):
        # degree: 원하는 각도 (음수는 반시계, 양수는 시계 방향)
        # angle_min, angle_increment는 라디안 단위
        
        # 원하는 각도 주변의 인덱스 검색 (e.g. 70도 주변 ±1도)
        for i, data in enumerate(self.msg.ranges):
            # i번째 데이터의 각도 (도 단위)
            current_degree = (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
            
            # 원하는 각도(degree) ±1도 이내이고, 유효한 거리(0.6m 미만)가 있는 경우
            if (degree - 1 <= current_degree <= degree + 1) and (0 < self.msg.ranges[i] <= 0.6):
                real_data = self.msg.ranges[i]
                return real_data
                
        return None # 해당 각도에 유효한 데이터가 없는 경우

    # 7. 이동 제어 메서드
    def move_control(self):
        
        if self.condition == "forward":
            self.speed = self.default_speed
            self.angle = 0
        elif self.condition == "close":
            print("벽과의 거리가 60cm 보다 작습니다.")
            if self.DIRECTION == LEFT:
                # 우회전
                self.angle = self.default_angle / (min(self.distance) * 10)
            elif self.DIRECTION == RIGHT:
                # 좌회전
                self.angle = self.default_angle / (min(self.distance) * 10)
        elif self.condition == "far":
            print("벽과의 거리가 60cm 보다 멉니다.")
            if self.DIRECTION == LEFT:
                # 좌회전 (벽 쪽으로)
                self.angle = -(self.default_angle) * 2
            elif self.DIRECTION == RIGHT:
                # 우회전 (벽 쪽으로)
                self.angle = self.default_angle * 2
        elif self.condition == "maintaining":
            self.maintain_direction()
        elif self.condition == "obstacle":
            self.obstacle_motion()
            
            # 장애물 회피 후 리스트 초기화
            self.obstacle_data_idx = []
            self.obstacle_data_range = []
            self.distance = []

    # 8. 메인 루프 메서드
    def main(self):
        if self.is_scan:
            self.LiDAR_scan()
            self.judge_distance()
            self.move_control()
            
            # 속도 및 각속도 설정
            self.cmd_vel_msg.linear.x = self.speed
            self.cmd_vel_msg.angular.z = self.angle
            
            # 메시지 발행
            self.pub.publish(self.cmd_vel_msg)
            
            self.is_scan = False

# Check if this is the main module that is being run
if __name__ == '__main__':
    # Limo_obstacle_avoidance 클래스의 인스턴스 생성 (LEFT 방향으로 벽 따르기)
    limo_wall_following = Limo_wall_following(LEFT) 
    
    # Start a loop that will continue until ROS is shutdown
    try:
        while not rospy.is_shutdown():
            limo_wall_following.main()
            
    # Catch the ROSInterruptException to ensure a clean exit
    except rospy.ROSInterruptException:
        # Call the main method of the Limo_obstacle_avoidance class
        # limo_wall_following.main() # 이 줄은 주석으로 처리하거나 삭제하는 것이 맞습니다.
        pass