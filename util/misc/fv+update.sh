#!/bin/bash
#\file    fv+update.sh
#\brief   Script to update FV+ software.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.21, 2023

answer_yes='n'
function print_usage
{
  echo "Usage: $0 [-y]"
  echo "  -y Answer yes to all queries."
}
while getopts 'y' flag; do
  case "${flag}" in
    y) answer_yes='y' ;;
    *) print_usage
       exit 1 ;;
  esac
done

function ask
{
  if [ "${answer_yes}" == "y" ]; then return 0; fi
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
  rosmake ay_util_msgs ay_util fingervision
fi
