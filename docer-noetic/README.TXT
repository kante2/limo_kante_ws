실행방법 :: 1101

cd /home/agilex/limo_kante_ws/docer-noetic

# 이미지 빌드
docker-compose build

# 컨테이너 실행
docker-compose up -d

# 접속 후 빌드
docker exec -it limo-noetic-nano bash
source /opt/ros/noetic/setup.bash
cd /root/ws
# (처음 한 번은 캐시 충돌 피하려면 build/devel/install 지우고 시작)
rm -rf build devel install
catkin_make
source devel/setup.bash
