xhost +si:localuser:root


cd /home/kante/limo_ws/src/limo_ros/docker_pc


# (선택) 먼저 빌드만
docker compose -f docker-compose.pc.yaml build

# 백그라운드 실행
docker compose -f docker-compose.pc.yaml up -d

# 컨테이너 상태
docker ps --filter "name=limo-noetic-pc"

# 로그 확인
docker logs --tail=100 limo-noetic-pc

# 셸 접속
docker exec -it limo-noetic-pc bash
