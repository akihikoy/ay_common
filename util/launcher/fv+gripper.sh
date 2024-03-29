#!/bin/bash
#\file    fv+gripper.sh
#\brief   FV+Gripper launcher script.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.03, 2022

AYCDIR=${HOME}/ros_ws/ay_tools/ay_common/util/launcher
if [ -f ${HOME}/fv+config.sh ]; then
  echo "Loading ${HOME}/fv+config.sh"
  . ${HOME}/fv+config.sh
elif [ -f ${AYCDIR}/fv+config.sh ]; then
  echo "Loading ${AYCDIR}/fv+config.sh"
  . ${AYCDIR}/fv+config.sh
else
  echo "Config file not found: fv+config.sh"
fi

. /opt/ros/$ROS_DISTR/setup.bash
. ~/catkin_ws/devel/setup.bash
. /opt/ros/$ROS_DISTR/share/rosbash/rosbash

if [ -z "$ROS_MASTER_URI" ]; then
  export ROS_MASTER_URI=http://localhost:11311
fi
if [ -z "$ROS_IP" ]; then
  export ROS_IP=127.0.0.1
fi
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$HOME/ros_ws:${HOME}/prg/ay_test/ros
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"

rosrun fv_gripper_ctrl ctrl_panel.py -gripper_type=${GRIPPER_TYPE} -dxl_dev=${DXL_DEV} ${FVG_OPTS} $@
