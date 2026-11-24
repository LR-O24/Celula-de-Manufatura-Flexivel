#!/bin/bash
# Notebook B (estação TM)
export IP_A=192.168.0.10
export ROS_MASTER_URI=http://$IP_A:11311
export ROS_IP=192.168.0.12
source /opt/ros/noetic/setup.bash
source ~/celula_ws/devel/setup.bash
