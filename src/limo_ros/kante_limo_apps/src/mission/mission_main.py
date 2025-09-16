import rospy
from enum import IntEnum
from geometry_msgs.msg import Twist
from kante_limo_apps.mission.mission_1 import Mission1
from kante_limo_apps.mission.mission_2 import Mission2

class Mode(IntEnum):
    M1 = 1
    M2 = 2

class MissionMain:
    def __init__(self):
        # 주기
        self.rate_hz = rospy.get_param("~rate_hz", 90)
        self.rate = rospy.Rate(self.rate_hz)

        # 시작 플래그/초기 모드 (필요시 파라미터로도 제어 가능)
        self.start_flag = rospy.get_param("~start_flag", True)
        start_mode = int(rospy.get_param("~start_mode", 1))
        self.mission_mode = Mode.M1 if start_mode != 2 else Mode.M2

        # 외부 상태 공급자(예: 스톱라인 인식 결과) — 필요하면 콜백으로 갱신
        self._stop_line = False
        # 예: /some_stop_line 토픽 구독해서 self._stop_line 갱신하도록 확장하면 됨

        # 미션 인스턴스 (필요 리소스/콜백 주입)
        self.missions = {
            Mode.M1: Mission1(stop_line_provider=lambda: self._stop_line),
            Mode.M2: Mission2(stop_line_provider=lambda: self._stop_line),
        }

        # 안전 정지 퍼블리셔
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._finished = False

    # 필요시 외부에서 스톱라인 갱신할 때 쓸 메서드(토픽 콜백 등에서 호출)
    def set_stop_line(self, val: bool):
        self._stop_line = bool(val)

    def stop_robot(self, duration=0.5):
        """안전정지: duration 동안 0속도 퍼블리시"""
        t_end = rospy.Time.now() + rospy.Duration.from_sec(duration)
        zero = Twist()
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            self.cmd_pub.publish(zero)
            self.rate.sleep()

    def run(self):
        while not rospy.is_shutdown():
            if not self.start_flag:
                self.rate.sleep()
                continue

            if self._finished:
                # 완료 후에도 주기적으로 정지 명령 유지(원하면 제거)
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            try:
                mission = self.missions.get(self.mission_mode, None)
                if mission is None:
                    rospy.logerr(f"[MissionMain] Unknown mode: {self.mission_mode}")
                    self._finished = True
                    continue

                done = mission.step()      # 각 미션이 True/False 반환

                if done:
                    # 전이 규칙: 1 → 2 → 종료
                    if self.mission_mode == Mode.M1:
                        rospy.loginfo("Mission 1 complete → switching to Mission 2")
                        self.stop_robot(0.5)  # 전이 시 잠깐 정지
                        self.mission_mode = Mode.M2
                    elif self.mission_mode == Mode.M2:
                        rospy.loginfo("Mission 2 complete → All done")
                        self.stop_robot(1.0)
                        self._finished = True

            except Exception as e:
                rospy.logerr_throttle(1.0, f"[MissionMain] error: {e}")

            self.rate.sleep()
