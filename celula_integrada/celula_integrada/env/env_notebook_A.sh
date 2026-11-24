#!/bin/bash
# Notebook A (MASTER): roscore + Maestro. Uso: source env/env_notebook_A.sh
export IP_A=192.168.0.10
export ROS_MASTER_URI=http://$IP_A:11311
export ROS_IP=$IP_A
source /opt/ros/noetic/setup.bash
source ~/celula_ws/devel/setup.bash
