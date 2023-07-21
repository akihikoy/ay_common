#!/bin/bash
#\file    fv+update.sh
#\brief   Script to update FV+ software.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.21, 2023

function ask
{
  while true; do
    echo -n '  (y|n) > '
    read s
    if [ "$s" == "y" ];then return 0; fi
    if [ "$s" == "n" ];then return 1; fi
  done
}

cd ~/ros_ws
rosws update

echo 'Re-compile?'
if ask; then
  rosmake ay_util_msgs
  rosmake ay_util
  rosmake fingervision
fi
