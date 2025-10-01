#!/usr/bin/env python3
import sys, select, termios, tty
import rospy
from geometry_msgs.msg import Twist

HELP = """
LIMO Simple Teleop (cmd_vel -> {topic})

키:
  i / k   : 전진 / 후진 (선형속도 단계 변경)
  j / l   : 좌회전 / 우회전 (각속도 단계 변경)
  k 또는 Space : 정지
  q / z   : 최대 선형속도 증가 / 감소
  e / c   : 최대 각속도 증가 / 감소
  Ctrl-C  : 종료

현재 최대치: lin={lin_max:.2f} m/s, ang={ang_max:.2f} rad/s
"""

def get_key(timeout=0.05):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    rospy.init_node("limo_simple_teleop")
    topic = rospy.get_param("~cmd_vel_topic", "/limo/cmd_vel")
    rate_hz = rospy.get_param("~rate", 20)
    step_lin = rospy.get_param("~step_linear", 0.05)     # m/s per key
    step_ang = rospy.get_param("~step_angular", 0.10)    # rad/s per key
    max_lin = rospy.get_param("~max_linear", 0.60)       # m/s
    max_ang = rospy.get_param("~max_angular", 1.20)      # rad/s

    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rate = rospy.Rate(rate_hz)

    settings = termios.tcgetattr(sys.stdin)
    lin = 0.0
    ang = 0.0

    try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP.format(topic=topic, lin_max=max_lin, ang_max=max_ang))
        while not rospy.is_shutdown():
            key = get_key(0.05)
            if key:
                if key == 'i':
                    lin = min(lin + step_lin,  max_lin)
                elif key == 'k':
                    lin = max(lin - step_lin, -max_lin)
                elif key == 'j':
                    ang = min(ang + step_ang,  max_ang)
                elif key == 'l':
                    ang = max(ang - step_ang, -max_ang)
                elif key in ('k', ' '):
                    lin, ang = 0.0, 0.0
                elif key == 'q':
                    max_lin = min(max_lin + 0.05, 2.0)
                    print(f"\nmax_linear = {max_lin:.2f} m/s")
                elif key == 'z':
                    max_lin = max(max_lin - 0.05, 0.05)
                    print(f"\nmax_linear = {max_lin:.2f} m/s")
                elif key == 'e':
                    max_ang = min(max_ang + 0.1, 6.0)
                    print(f"\nmax_angular = {max_ang:.2f} rad/s")
                elif key == 'c':
                    max_ang = max(max_ang - 0.1, 0.1)
                    print(f"\nmax_angular = {max_ang:.2f} rad/s")
                elif key in ('\x03', '\x1b'):  # Ctrl-C or ESC
                    break
                # 상태 줄 출력
                print(f"\rlin={lin:.2f} m/s  ang={ang:.2f} rad/s   ", end="")
                sys.stdout.flush()

            # 항상 현재 값을 퍼블리시 (키 안 눌러도 유지)
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            pub.publish(msg)
            rate.sleep()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # 정지 명령 한번 내리고 종료
        stop = Twist()
        pub.publish(stop)
        print("\nBye")

if __name__ == "__main__":
    main()
