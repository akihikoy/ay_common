#!/bin/bash
#\file    fv+ur-demo.sh
#\brief   FV+UR-demo launcher script.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.03, 2022

#ROS Configuration
#ROS_DISTR=kinetic
ROS_DISTR=melodic
. /opt/ros/$ROS_DISTR/setup.bash
. ~/catkin_ws/devel/setup.bash
. /opt/ros/$ROS_DISTR/share/rosbash/rosbash

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$HOME/ros_ws:${HOME}/prg/ay_test/ros

rosrun ay_util ur_panel.py -robot_code=UR3e125hzThG -dxl_dev=USB0
