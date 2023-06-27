# Nav2-RRTStar
Implementation of RRT* algorithm in Nav2 for ros2 humble. Implementation is using naive aproach to look for nearest neighbours so it is very slow with big number of nodes or big maps. It was written as a way of getting familiar with Nav2. Should not be used in any serious aplication.
## Runnig examples
### Required:
1. navigation2
2. turtlebot3_gazebo
### After building the package run:
1. export TURTLEBOT3_MODEL=waffle
2. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
3. source install/setup.bash
### Then run one of 2 examples:
1. ros2 launch nav2_rrtstar_planner tb3_world_demo.launch.py
2. ros2 launch nav2_rrtstar_planner tb3_house_demo.launch.py
