xhost +local:docker

cd /home/kante/Documents/limo_kante_ws/docker_melodic_gpu  # (폴더 이름은 그대로 써도 됨)
docker rm -f noetic_gpu_gui 2>/dev/null || true

USER_ID=$(id -u) GROUP_ID=$(id -g) docker compose -f docker-compose.yaml up -d --build
docker compose -f docker-compose.yaml ps
