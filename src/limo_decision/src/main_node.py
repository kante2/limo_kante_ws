#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool

from limo_lane_mission import LimoLaneDecision
from limo_wall_follow_mission import LimoWallFollowDecision


class MissionManager:
    def __init__(self):
        rospy.init_node("autorace_main_decision")

        self.lane = LimoLaneDecision()
        self.wall = LimoWallFollowDecision()

        self.wall_valid = False
        wall_valid_topic = rospy.get_param("~wall_valid_topic", "/wall_follow/valid")
        rospy.Subscriber(wall_valid_topic, Bool, self.wall_valid_cb, queue_size=1)

        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 30.0))
        self.prev_state = None

    def wall_valid_cb(self, msg: Bool):
        self.wall_valid = msg.data

    def run(self):
        while not rospy.is_shutdown():
            state = "WALL" if self.wall_valid else "LANE"
            if state != self.prev_state:
                rospy.loginfo("[main_node] Mission changed -> %s", state)
                self.prev_state = state

            if state == "WALL":
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
