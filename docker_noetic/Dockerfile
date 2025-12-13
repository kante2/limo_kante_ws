FROM ros:noetic-ros-base-focal

ARG USERNAME=kante
ARG USER_ID=1000
ARG GROUP_ID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# ✅ 컨테이너 워크스페이스 경로
ENV WS_DIR=/limo_kante_ws

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim nano build-essential \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-wstool \
    python3-catkin-tools \
    # GUI (rviz / rqt)
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-tf2-tools \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# 호스트 UID/GID 맞춰 유저 생성 (권한 꼬임 방지)
RUN groupadd -g ${GROUP_ID} ${USERNAME} || true && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME} || true

RUN mkdir -p ${WS_DIR}/src && chown -R ${USER_ID}:${GROUP_ID} ${WS_DIR}

# entrypoint
RUN cat > /entrypoint.sh <<'EOF'
#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
if [ -f /limo_kante_ws/devel/setup.bash ]; then
  source /limo_kante_ws/devel/setup.bash
fi
exec "$@"
EOF
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR ${WS_DIR}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
