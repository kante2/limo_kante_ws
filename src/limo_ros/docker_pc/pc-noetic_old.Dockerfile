# PC(AMD64)용: Gazebo 포함
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive TZ=Asia/Seoul
SHELL ["/bin/bash","-lc"]

# 기본 개발 도구 + ROS 패키지 (limo_ros 빌드/테스트용)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake python3-pip python3-rosdep \
    ros-noetic-navigation \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
    ros-noetic-xacro ros-noetic-imu-tools \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN rosdep init 2>/dev/null || true && rosdep update

# 워크스페이스 디렉터리(호스트 ~/limo_ws 가 여기에 마운트됨)
RUN mkdir -p /root/ws/src
WORKDIR /root/ws

# 편의 환경
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/devel/setup.bash"  >> /root/.bashrc
