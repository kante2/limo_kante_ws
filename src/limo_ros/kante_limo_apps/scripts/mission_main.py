#!/usr/bin/env python3
import rospy
from kante_limo_apps.mission.mission_main import MissionMain

if __name__ == "__main__":
    rospy.init_node("mission_main")
    MissionMain().run()
