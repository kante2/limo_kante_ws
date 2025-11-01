#!/usr/bin/env bash
set -e

# ROS 환경
source /opt/ros/noetic/setup.bash

# rosdep 업데이트 (네트워크 이슈 시 오류 무시)
rosdep update || true

# 호스트에서 마운트된 src 의존 설치
if [ -d /root/ws/src ]; then
  rosdep install --from-paths /root/ws/src --ignore-src -r -y || true
fi

############################################
# (A) YDLidar-SDK 자동 설치 (있을 때만)
############################################
if [ -d /root/ws/src/YDLidar-SDK ]; then
  echo "[init_ws] Found YDLidar-SDK. Building & Installing to /opt/ydlidar ..."
  pushd /root/ws/src/YDLidar-SDK >/dev/null
  rm -rf build && mkdir -p build && cd build

  # 표준 lib/cmake 트리로 설치
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/ydlidar \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_INSTALL_INCLUDEDIR=include \
    -DBUILD_SHARED_LIBS=ON
  make -j$(nproc)
  make install

  # 링커/패키지 경로 등록
  echo "/opt/ydlidar/lib" >/etc/ld.so.conf.d/ydlidar.conf
  ldconfig

  # CMake 패키지 경로 (둘 중 존재하는 쪽 사용)
  if [ -d /opt/ydlidar/lib/cmake/ydlidar_sdk ]; then
    export ydlidar_sdk_DIR=/opt/ydlidar/lib/cmake/ydlidar_sdk
  elif [ -f /opt/ydlidar/cmake/ydlidar_sdkConfig.cmake ]; then
    export ydlidar_sdk_DIR=/opt/ydlidar/cmake
  fi

  # 환경 영구 반영
  grep -q "ydlidar_sdk_DIR" ~/.bashrc || {
    if [ -n "$ydlidar_sdk_DIR" ]; then
      echo "export ydlidar_sdk_DIR=$ydlidar_sdk_DIR" >> ~/.bashrc
    fi
  }
  grep -q "/opt/ydlidar" ~/.bashrc || echo "export CMAKE_PREFIX_PATH=/opt/ydlidar:\$CMAKE_PREFIX_PATH" >> ~/.bashrc
  grep -q "/opt/ydlidar/lib" ~/.bashrc || echo "export LD_LIBRARY_PATH=/opt/ydlidar/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
  popd >/dev/null
fi

############################################
# (B) 워크스페이스 빌드
############################################
mkdir -p /root/ws/src
pushd /root/ws >/dev/null

# catkin toplevel 보장
if [ ! -f /root/ws/src/CMakeLists.txt ]; then
  ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake /root/ws/src/CMakeLists.txt
fi

# 이전 빌드 잔여물 제거 (호스트-컨테이너 경로 불일치 방지)
rm -rf build devel
catkin_make

# 다음 셸 접속 시 자동 로드
grep -q "/opt/ros/noetic/setup.bash" ~/.bashrc || echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
grep -q "/root/ws/devel/setup.bash" ~/.bashrc   || echo "source /root/ws/devel/setup.bash" >> ~/.bashrc

popd >/dev/null

# 셸 유지 (컨테이너 계속 켜두기)
exec bash -lc "source /opt/ros/noetic/setup.bash; source /root/ws/devel/setup.bash; sleep infinity"
