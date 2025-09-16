# PC(AMD64)용: Gazebo 포함 Noetic 데스크탑 풀
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Seoul \
    LANG=C.UTF-8

# 필수 도구 + ros_control + Gazebo 제어 플러그인 + OpenGL 유틸
# (mesa-utils: glxinfo, GLVND/Mesa 런타임. NVIDIA는 호스트 드라이버를 사용)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-pip python3-rosdep \
      ros-noetic-teleop-twist-keyboard \
      ros-noetic-xacro \
      ros-noetic-robot-state-publisher \
      ros-noetic-joint-state-publisher \
      ros-noetic-ros-control \
      ros-noetic-ros-controllers \
      ros-noetic-gazebo-ros-control \
      libglvnd0 libgl1-mesa-glx libglu1-mesa mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# 워크스페이스 기본 디렉터리
RUN mkdir -p /root/ws/src
WORKDIR /root/ws

# 편의: bash 진입 시 ROS 및 ws 자동 소스(있을 때만)
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/ws/devel/setup.bash ] && source /root/ws/devel/setup.bash' >> /root/.bashrc

# (옵션) rosdep 초기화는 컨테이너 실행 후 1회만 수행 권장
# RUN rosdep init || true && rosdep update || true

# 기본 엔트리포인트는 베이스 이미지 것을 사용
# 컨테이너는 compose의 ["tail","-f","/dev/null"]로 유지됨
