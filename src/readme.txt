# How to run the mission scripts

## 1. Sensor preprocessing (perception launch)
```
cd /limo_kante_ws/src
./sensor_preprocessing.sh
```
- sources `/limo_kante_ws/devel/setup.bash`
- runs `roslaunch limo_perception lane_and_labacorn_perception.launch`

## 2. Main decision node
Open another terminal after the perception stack is up:
```
cd /limo_kante_ws/src
./main_node.sh
```
- sources `/limo_kante_ws/devel/setup.bash`
- runs `rosrun limo_decision main_node_2.py`

두 스크립트 모두 같은 디렉터리에서 실행하면 되고, 추가 인수 없이 각각 인지/디시전 노드를 띄웁니다.
