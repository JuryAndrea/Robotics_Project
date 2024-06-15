#!/usr/bin/env bash

cd $HOME/dev_ws
colcon build

ros2 launch robomaster_ros s1.launch name:=RM0001 tof_0:=true &

ros2 run robomaster_proj robomapper_node2 &

wait
