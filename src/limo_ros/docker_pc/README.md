# PC Noetic Dev Container (docker_pc)


# build
이미지: limo-noetic:pc, 
컨테이너: limo-noetic-pc)으로 
다시 생성하는 명령
 (현재 경로: ~/limo_ws/src/limo_ros/docker_pc)

0) (호스트에서 1회) X11 허용
xhost +local:docker

1) 리포 루트로 이동
cd ~/limo_ws/src/limo_ros

2) 이미지 재빌드
docker compose -f docker_pc/docker-compose.pc.yaml build --no-cache

3) 컨테이너 생성/기동
docker compose -f docker_pc/docker-compose.pc.yaml up -d --force-recreate --compatibility

4) 확인 & 접속
docker ps -a --filter "name=limo-noetic-pc"
docker exec -it limo-noetic-pc bash


만약 unknown runtime nvidia가 뜨면 호스트에 nvidia-container-toolkit 설치/설정이 필요.
nvidia-smi / glxinfo -B 확인하려면 컨테이너 들어가서 실행

nvidia-smi || true
glxinfo -B | egrep 'OpenGL vendor|renderer|version' || true

# error 
에러 수정 참고

unknown or invalid runtime name: nvidia 는 Docker 데몬에 nvidia 런타임이 등록 안 된 상태에서 compose가 runtime: nvidia를 요구해서 난 에러예요. 두 가지 중 하나로 바로 해결할 수 있어요.

방법 A) runtime: nvidia 그대로 쓰기 (툴킷 설치)

툴킷 설치/등록(호스트):

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker


런타임 등록 확인:

docker info | grep -A3 Runtimes   # nvidia 가 보여야 정상


다시 기동:

cd ~/limo_ws/src/limo_ros
docker compose -f docker_pc/docker-compose.pc.yaml up -d --force-recreate
