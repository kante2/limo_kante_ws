#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool

from limo_lane_mission import LimoLaneDecision
from limo_wall_follow_mission import LimoWallFollowDecision
from limo_obstacle_mission import LimoObstacleDecision


class MissionManager:
    def __init__(self):
        rospy.init_node("autorace_main_decision")

        # 각 미션 decision 객체들(각각 step 함수만 제공)
        self.lane = LimoLaneDecision()
        self.wall = LimoWallFollowDecision()
        self.obs  = LimoObstacleDecision()

        # ===== valid 신호들 =====
        self.wall_valid = False
        self.obstacle_valid = False

        wall_valid_topic = rospy.get_param("~wall_valid_topic", "/wall_follow/valid")
        obstacle_valid_topic = rospy.get_param("~obstacle_valid_topic", "/obstacle/valid")

        rospy.Subscriber(wall_valid_topic, Bool, self.wall_valid_cb, queue_size=1)
        rospy.Subscriber(obstacle_valid_topic, Bool, self.obstacle_valid_cb, queue_size=1)

        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 30.0))
        self.prev_state = None

        rospy.loginfo("[main_node] wall_valid_topic=%s, obstacle_valid_topic=%s",
                      wall_valid_topic, obstacle_valid_topic)

    def wall_valid_cb(self, msg: Bool):
        self.wall_valid = msg.data

    def obstacle_valid_cb(self, msg: Bool):
        self.obstacle_valid = msg.data

    def run(self):
        while not rospy.is_shutdown():
            # ===== 우선순위: OBSTACLE > WALL > LANE =====
            if self.obstacle_valid:
                state = "OBSTACLE"
            elif self.wall_valid:
                state = "WALL"
            else:
                state = "LANE"

            if state != self.prev_state:
                rospy.loginfo("[main_node] Mission changed -> %s", state)
                self.prev_state = state

            # ===== 해당 미션 step 1회 수행 =====
            if state == "OBSTACLE":
                self.obs.mission_obstacle_step()
            elif state == "WALL":
                self.wall.mission_wall_follow_step()
            else:
                self.lane.mission_lane_step()

            self.loop_rate.sleep()


def main():
    manager = MissionManager()
    manager.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
