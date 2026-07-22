#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/zeronetech/ros2_ws/install/setup.bash

ros2 launch robots spawn_robot.launch.py &
sleep 3

ros2 run laser_detection obstacle_extractor_node &
sleep 3

ros2 run shared_world_model shared_world_model_node &
sleep 3

wait
