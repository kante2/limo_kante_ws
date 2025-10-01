import rospy
from geometry_msgs.msg import Twist

class Mission1:
    """
    미션 1: 예) 기본 차선 주행/특정 이벤트 대기
    완료 조건을 만족하면 True 반환
    """
    def __init__(self, stop_line_provider):
        self._stop_line = stop_line_provider
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._done = False
        self._last = rospy.Time.now()

        # TODO: 필요하면 여기서 perception/control 모듈 연결
        # self.detector = ...
        # self.pid = ...

    def step(self) -> bool:
        if self._done:
            return True

        # 예시 로직 (자리표시자): 전진
        cmd = Twist()
        cmd.linear.x = 0.2
        self.pub.publish(cmd)

        # 완료 조건 예시: stop_line True가 일정 시간 유지
        now = rospy.Time.now()
        if self._stop_line():
            if (now - self._last).to_sec() > 0.3:
                self._done = True
        else:
            self._last = now

        return self._done
