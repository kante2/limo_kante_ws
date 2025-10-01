## kante upload 
### 250912 upload!


### real robot _ limo(jetson nano)

roslaunch limo_base limo_base.launch


### simulator using gazebo
### limo_gazebo_sim / limo_gazebo_utils

roslaunch limo_gazebo_sim limo_ackerman.launch 

roslaunch limo_gazebo_utils limo_teleop.launch

rosrun limo_gazebo_utils limo_lidar_view.py
rosrun limo_gazebo_utils limo_camera_view.py


### ydlidar_ros_driver pb solve

# (0) 워크스페이스 클린(선택)
cd /root/ws
rm -rf build devel

# (1) YDLidar SDK 설치
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install

# (2) 