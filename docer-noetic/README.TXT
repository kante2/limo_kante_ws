# X11 권한 허용
xhost +si:localuser:root

# compose 있는 디렉토리로 이동
cd /home/agilex/limo_kante_ws
docker compose up -d

# 컨테이너 접속
docker exec -it limo-noetic-nano bash
# (컨테이너 안)
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make && source devel/setup.bash
