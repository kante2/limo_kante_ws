Build & run overview
====================

1. catkin_make
   - Builds all packages in the workspace (run at /root/ws).

2. source devel/setup.bash
   - Loads the workspace environment into the current shell.

3. roslaunch limo_perception lane_and_labacorn_perception.launch
   - Starts both camera lane perception and LiDAR labacorn perception nodes.

4. rosrun limo_decision autorace_main_node
   - Runs the decision main node that consumes the perception topics.

Run the commands in separate terminals (or backgrounded) to keep the full pipeline active.
