# how to use limo

## 1. Real robot (Jetson Nano)
```
roslaunch limo_base limo_base.launch // 또는  bringup머시기 하면 됨!
```

# --실제 로봇은 여기!
## 3. Perception + Decision 실행
터미널을 두 개 열고 각각 아래 명령을 실행합니다. (워크스페이스: `/root/ws`)

터미널 1 – 카메라 차선 + LiDAR 라바콘 퍼셉션 런치
```
cd /root/ws
source devel/setup.bash      # 필요 시
roslaunch limo_perception lane_and_labacorn_perception.launch
```

터미널 2 – 메인 미션 디시전 노드 (lane/labacorn 전환 포함)
```
cd /root/ws
source devel/setup.bash      # 필요 시
rosrun limo_decision main_node
```

이후 각 노드는 `/labacorn/target`, `/labacorn_detected`, `/cmd_vel`, `/commands/motor/speed`, `/commands/servo/position` 등 관련 토픽을 주고받으며 주행을 수행합니다.
