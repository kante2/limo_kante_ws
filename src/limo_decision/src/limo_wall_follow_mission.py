#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Decision node for wall-follow mission.
Perception node publishes wall-related measurements (condition, min distance, etc.).
This node converts them into /cmd_vel commands.
"""

import math
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

LEFT = "LEFT"
RIGHT = "RIGHT"


class LimoWallFollowDecision:
    def __init__(self):

        self.direction = rospy.get_param("~direction", LEFT).upper()
        self.default_speed = rospy.get_param("~default_speed", 0.15)
        self.default_angle = rospy.get_param("~default_angle", 0.2)

        cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=3)

        condition_topic = rospy.get_param("~condition_topic", "/wall_follow/condition")
        min_distance_topic = rospy.get_param("~min_distance_topic", "/wall_follow/min_distance")
        side_dist_topic = rospy.get_param("~side_dist_topic", "/wall_follow/side_distances")
        turn_angle_topic = rospy.get_param("~turn_angle_topic", "/wall_follow/turn_angle")
        valid_topic = rospy.get_param("~valid_topic", "/wall_follow/valid")

        rospy.Subscriber(condition_topic, String, self.condition_cb, queue_size=1)
        rospy.Subscriber(min_distance_topic, Float32, self.min_distance_cb, queue_size=1)
        rospy.Subscriber(side_dist_topic, Float32MultiArray, self.side_dist_cb, queue_size=1)
        rospy.Subscriber(turn_angle_topic, Float32, self.turn_angle_cb, queue_size=1)
        rospy.Subscriber(valid_topic, Bool, self.valid_cb, queue_size=1)

        self.condition = "forward"
        self.min_distance = 0.0
        self.left70: Optional[float] = None
        self.left80: Optional[float] = None
        self.right70: Optional[float] = None
        self.right80: Optional[float] = None
        self.turn_angle = 0.0
        self.data_valid = False

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "[wall_follow_decision] direction=%s cmd_topic=%s",
            self.direction,
            cmd_topic,
        )

    # --------- Subscribers ---------
    def condition_cb(self, msg: String):
        self.condition = msg.data or "forward"

    def min_distance_cb(self, msg: Float32):
        self.min_distance = msg.data

    def side_dist_cb(self, msg: Float32MultiArray):
        data = msg.data or []
        self.left70 = data[0] if len(data) > 0 and data[0] > 0.0 else None
        self.left80 = data[1] if len(data) > 1 and data[1] > 0.0 else None
        self.right70 = data[2] if len(data) > 2 and data[2] > 0.0 else None
        self.right80 = data[3] if len(data) > 3 and data[3] > 0.0 else None

    def turn_angle_cb(self, msg: Float32):
        self.turn_angle = msg.data

    def valid_cb(self, msg: Bool):
        self.data_valid = msg.data

    def on_shutdown(self):
        self.cmd_pub.publish(Twist())

    # --------- Control helpers ---------
    def maintain_direction(self) -> float:
        if self.direction == LEFT:
            angle1 = self.left70
            angle2 = self.left80
        else:
            angle1 = self.right70
            angle2 = self.right80

        if angle1 is None or angle2 is None or angle1 == 0.0:
            return 0.0

        try:
            ratio = max(min(angle2 / angle1, 1.0), -1.0)
            theta = math.acos(ratio)
        except ValueError:
            return 0.0

        theta_deg = math.degrees(theta)
        default_theta = 0.28

        if 0.0 <= theta_deg <= 20.0:
            return (theta - default_theta) if self.direction == LEFT else -(theta - default_theta)

        if theta_deg > 20.0:
            return -(theta_deg - default_theta) if self.direction == LEFT else theta - default_theta

        return 0.0

    def obstacle_motion(self) -> Twist:
        cmd = Twist()
        angle = self.turn_angle if self.direction == LEFT else -self.turn_angle
        cmd.linear.x = -self.turn_angle / 10.0
        cmd.angular.z = angle
        return cmd

    def mission_wall_follow_step(self):
        if not self.data_valid:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        cond = self.condition

        if cond == "forward":
            cmd.linear.x = self.default_speed
            cmd.angular.z = 0.0
        elif cond == "close":
            if self.min_distance > 0.0:
                cmd.angular.z = self.default_angle / (self.min_distance * 10.0)
            cmd.linear.x = self.default_speed
        elif cond == "far":
            cmd.linear.x = self.default_speed
            cmd.angular.z = -(self.default_angle) * 2.0 if self.direction == LEFT else self.default_angle * 2.0
        elif cond == "maintaining":
            cmd.linear.x = self.default_speed
            cmd.angular.z = self.maintain_direction()
        elif cond == "obstacle":
            cmd = self.obstacle_motion()
        else:
            cmd.linear.x = self.default_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
