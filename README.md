
# 실행 가이드

## 1. 센서/인지 노드 실행
Perception 관련 launch 파일을 먼저 실행.

```bash
roslaunch limo_perception lane_and_labacorn_perception.launch
```

해당 launch는 카메라 차선 인지와 LiDAR 기반 라바콘/벽 인지를 모두 킨다.
(참고로 카메라만, 또는 라이다만 퍼블리시 하고 싶다면 rosrun (해당 파일) 로 실행 )

## 2. 메인 결정 노드 실행
Perception이 준비되면 decision 노드를 실행.

```bash
rosrun limo_decision main_node
```

메인 노드는 각 인지 토픽을 구독해 미션 상태를 결정하고 `/cmd_vel` 또는 모터/서보 토픽을 퍼블리시.
