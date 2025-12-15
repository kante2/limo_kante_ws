
# 실행 가이드

## 1. 센서/인지 노드 실행
Perception 관련 launch 파일을 먼저 실행.

```bash
cd /limo_kante_ws/src
./sensor_preprocessing.sh
```

해당 스크립트는 `/limo_kante_ws/devel/setup.bash`를 source 한 뒤 `roslaunch limo_perception lane_and_labacorn_perception.launch`를 실행해 카메라 차선 + LiDAR 장애물 인지를 동시에 띄운다.

## 2. 메인 결정 노드 실행
Perception이 준비되면 decision 노드를 실행.

```bash
cd /limo_kante_ws/src
./main_node.sh
```

스크립트는 동일하게 환경을 로드한 후 `rosrun limo_decision main_node_2.py`를 호출한다. 메인 노드는 각 인지 토픽을 구독해 미션 상태를 결정하고 `/cmd_vel` 또는 모터/서보 토픽을 퍼블리시.
