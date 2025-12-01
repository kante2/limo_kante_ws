#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from std_msgs.msg import Bool, Float32, Float64


class LimoLabacornDecision:
    """
    Labacorn 미션 디시전 노드.
    - /labacorn/target (Float32)와 /labacorn_detected (Bool)을 입력으로 사용
    - /commands/motor/speed, /commands/servo/position으로 저수준 제어 퍼블리시
    - mission_labacorn_step()을 주기적으로 호출하여 동작
    """

    def __init__(self):
        self.target_topic = rospy.get_param("~target_topic", "/labacorn/target")
        self.detected_topic = rospy.get_param("~detected_topic", "/labacorn_detected")
        self.speed_topic = rospy.get_param("~speed_topic", "/commands/motor/speed")
        self.servo_topic = rospy.get_param("~servo_topic", "/commands/servo/position")
        self.control_rate = rospy.get_param("~control_rate", 15.0)

        # 속도/조향 파라미터
        self.follow_speed = rospy.get_param("~follow_speed", 1100.0)
        self.min_speed = rospy.get_param("~min_speed", 900.0)
        self.k_yaw = rospy.get_param("~k_yaw", 1.2)
        self.servo_center = rospy.get_param("~servo_center", 0.545)
        self.left_scale = rospy.get_param("~left_scale", 25.0)
        self.right_scale = rospy.get_param("~right_scale", 20.0)
        self.keep_forward_on_loss = rospy.get_param("~keep_forward_on_loss", True)

        self.latest_yaw = 0.0
        self.target_valid = False

        self.pub_speed = rospy.Publisher(self.speed_topic, Float64, queue_size=1)
        self.pub_servo = rospy.Publisher(self.servo_topic, Float64, queue_size=1)

        rospy.Subscriber(self.target_topic, Float32, self._target_cb, queue_size=1)
        rospy.Subscriber(self.detected_topic, Bool, self._detected_cb, queue_size=1)

        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / max(1e-3, self.control_rate)),
            self._timer_cb,
        )
        rospy.on_shutdown(self._shutdown_hook)

        rospy.loginfo(
            "[labacorn_decision] target=%s detected=%s -> speed:%s servo:%s",
            self.target_topic,
            self.detected_topic,
            self.speed_topic,
            self.servo_topic,
        )

    def _target_cb(self, msg: Float32):
        self.latest_yaw = msg.data

    def _detected_cb(self, msg: Bool):
        self.target_valid = msg.data

    def _timer_cb(self, _event):
        self.mission_labacorn_step()

    def _shutdown_hook(self):
        rospy.loginfo("[labacorn_decision] shutdown -> stop robot")
        stop_msg = Float64()
        stop_msg.data = 0.0
        servo_msg = Float64()
        servo_msg.data = self.servo_center
        self.pub_speed.publish(stop_msg)
        self.pub_servo.publish(servo_msg)

    def _publish_speed_servo(self, speed: float, servo: float):
        speed_msg = Float64()
        speed_msg.data = speed
        servo_msg = Float64()
        servo_msg.data = servo
        self.pub_speed.publish(speed_msg)
        self.pub_servo.publish(servo_msg)

    def publish_minimal_speed(self, move_forward: bool = True):
        if move_forward:
            self._publish_speed_servo(self.min_speed, self.servo_center)
            rospy.loginfo_throttle(
                1.0, "[labacorn_decision] no target -> slow forward (%.0f)", self.min_speed
            )
        else:
            self._publish_speed_servo(0.0, self.servo_center)
            rospy.loginfo_throttle(1.0, "[labacorn_decision] no target -> stop")

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def yaw_to_servo(self, yaw: float) -> float:
        yaw_deg = math.degrees(yaw)
        if yaw_deg < 0.0:
            steering = self.servo_center + (yaw_deg / self.left_scale) * self.servo_center
        else:
            steering = self.servo_center + (yaw_deg / self.right_scale) * (1.0 - self.servo_center)
        return self.clamp(steering, 0.0, 1.0)

    def mission_labacorn_step(self):
        if not self.target_valid:
            self.publish_minimal_speed(move_forward=self.keep_forward_on_loss)
            return

        yaw_cmd = self.latest_yaw * self.k_yaw
        steering = self.yaw_to_servo(yaw_cmd)
        self._publish_speed_servo(self.follow_speed, steering)

        rospy.loginfo_throttle(
            1.0,
            "[labacorn_decision] speed=%.0f steer=%.3f (yaw=%.3f rad, gain=%.2f)",
            self.follow_speed,
            steering,
            self.latest_yaw,
            self.k_yaw,
        )


def main():
    rospy.init_node("limo_labacorn_decision")
    LimoLabacornDecision()
    rospy.spin()


if __name__ == "__main__":
    main()
