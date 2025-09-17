#!/usr/bin/env python3
import rospy
from kante_limo_apps.sensor.lidar_avoid_person import LidarAvoidPerson

if __name__ == "__main__":
    rospy.init_node("lidar_avoid_person")

    node = LidarAvoidPerson(
        scan_topic = rospy.get_param("~scan_topic", "/scan"),
        cmd_topic  = rospy.get_param("~cmd_topic",  "/cmd_vel"),
        sector_deg = rospy.get_param("~sector_deg", 40.0),
        stop_dist  = rospy.get_param("~stop_dist",  0.60),
        slow_dist  = rospy.get_param("~slow_dist",  1.00),
        max_speed  = rospy.get_param("~max_speed",  0.25),
        slow_speed = rospy.get_param("~slow_speed", 0.10),
        turn_speed = rospy.get_param("~turn_speed", 0.25),
    )
    rospy.spin()
