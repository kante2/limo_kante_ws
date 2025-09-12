cmake_minimum_required(VERSION 3.0.2)
project(limo_gazebo_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python scripts 설치 (rosrun/roslaunch로 실행 가능해짐)
catkin_install_python(PROGRAMS
  scripts/limo_camera_view.py
  scripts/limo_lidar_view.py
  scripts/limo_teleop.py
  scripts/limo_vel_steer_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
