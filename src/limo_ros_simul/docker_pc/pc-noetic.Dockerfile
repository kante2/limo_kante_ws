# PC(AMD64)용: Gazebo 포함 Noetic 데스크탑 풀
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Seoul \
    LANG=C.UTF-8

# 필수 도구 + ros_control + Gazebo + OpenGL 유틸
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-pip python3-rosdep \
      ros-noetic-teleop-twist-keyboard \
      ros-noetic-xacro \
      ros-noetic-robot-state-publisher \
      ros-noetic-joint-state-publisher \
      ros-noetic-ros-control \
      ros-noetic-ros-controllers \
      ros-noetic-gazebo-ros-control \
      software-properties-common \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ★ RDNA3 위한 최신 Mesa (kisak PPA)
RUN add-apt-repository -y ppa:kisak/kisak-mesa && \
    apt-get update && apt-get install -y --no-install-recommends \
      libgl1-mesa-dri mesa-vulkan-drivers libdrm2 libgl1-mesa-glx libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# 워크스페이스
RUN mkdir -p /root/ws/src
WORKDIR /root/ws

# ROS 자동 source
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/ws/devel/setup.bash ] && source /root/ws/devel/setup.bash' >> /root/.bashrc
