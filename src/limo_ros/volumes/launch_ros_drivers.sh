#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash

# 컨테이너가 마스터
roscore & sleep 2

# TODO: 실제 드라이버/런치로 수정 (예시는 bringup)
# 예: roslaunch limo_bringup limo_start.launch
# 우선 컨테이너 유지
tail -f /dev/null
