cd /root/ws/src/limo_ros/docker_nano

# (1) 이미지 빌드 + 컨테이너 생성/기동
docker compose -f docker-compose.yaml up -d --build

# (2) 상태 확인
docker ps -a   # limo-noetic-nano 가 떠 있어야 함

# (3) 접속 (쉘 들어가기)
docker exec -it limo-noetic-nano bash

# (4) 컨테이너 쉘 진입
docker exec -it limo-noetic-nano bash