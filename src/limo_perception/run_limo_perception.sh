#!/usr/bin/env bash

# LIMO 퍼셉션 패키지 빌드 & 실행 스크립트
set -euo pipefail

usage() {
  cat <<'EOF'
사용법: ./run_limo_perception.sh [옵션]

옵션:
  --build       roslaunch 실행 전 catkin_make 수행 (빌드 스킵 시 생략)
  --skip-build  catkin_make 절대 수행하지 않음 (기본값)
  --help        도움말 출력

예)
  ./run_limo_perception.sh --build
  ./run_limo_perception.sh lane_node_name:=custom_lane labacorn_node_name:=custom_lidar

추가로 넘긴 인자는 그대로 roslaunch에 전달됩니다.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_FIRST=false
FORCE_SKIP=false

extra_args=()
while (($#)); do
  case "$1" in
    --build)
      BUILD_FIRST=true
      ;;
    --skip-build)
      FORCE_SKIP=true
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      extra_args+=("$1")
      ;;
  esac
  shift
done

if "${FORCE_SKIP}"; then
  BUILD_FIRST=false
fi

# ROS 기본 환경 불러오기
ROS_SETUP="/opt/ros/noetic/setup.bash"
if [[ -f "${ROS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
else
  echo "[경고] ${ROS_SETUP} 파일을 찾을 수 없습니다. ROS 환경이 설정되어 있는지 확인하세요." >&2
fi

cd "${WORKSPACE_ROOT}"

if "${BUILD_FIRST}"; then
  echo "[정보] catkin_make 실행 중..."
  catkin_make
fi

# 워크스페이스 오버레이 소스
if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_ROOT}/devel/setup.bash"
elif [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_ROOT}/install/setup.bash"
else
  echo "[경고] devel 또는 install setup을 찾을 수 없습니다. catkin_make를 먼저 실행했는지 확인하세요." >&2
fi

echo "[정보] lane_and_labacorn_perception.launch 실행"
exec roslaunch limo_perception lane_and_labacorn_perception.launch "${extra_args[@]}"
