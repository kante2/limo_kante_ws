import rospy
from geometry_msgs.msg import Twist

class Mission2:
    """
    미션 2: 예) 속도 변경/차선 변경/회피 등
    완료 조건을 만족하면 True 반환
    """
    def __init__(self, stop_line_provider):
        self._stop_line = stop_line_provider
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._done = False
        self._t0 = rospy.Time.now()

    def step(self) -> bool:
        if self._done:
            return True

        # 예시 로직 (자리표시자): 더 천천히 진행
        cmd = Twist()
        cmd.linear.x = 0.15
        self.pub.publish(cmd)

        # 완료 조건 예시: 일정 시간 경과 or 외부 이벤트
        if (rospy.Time.now() - self._t0).to_sec() > 5.0 or self._stop_line():
            self._done = True

        return self._done
