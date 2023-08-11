#!/bin/bash -x
#\file    fv+gripper-setup.sh
#\brief   Script to setup a Linux PC for FV+GripperKit.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.21, 2023

ros_bash_config="
###AUTOMATICALLY_ADDED_BY_THE_FV_SCRIPT###
#ROS Configuration
#ROS_DISTR=kinetic
ROS_DISTR=melodic
. /opt/ros/\$ROS_DISTR/setup.bash
. ~/catkin_ws/devel/setup.bash
. /opt/ros/\$ROS_DISTR/share/rosbash/rosbash

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
#export ROS_MASTER_URI=http://10.10.6.203:11311
#export ROS_IP=10.10.6.203
export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:\$HOME/ros_ws:\${HOME}/prg/ay_test/ros
"

answer_yes_admin='n'
answer_yes_user='n'
function print_usage
{
  echo "Usage: $0 [-y]"
  echo "  -y Answer yes to all queries about the system (admin) setup."
  echo "  -u Answer yes to all queries about the per-user setup."
}
while getopts 'uy' flag; do
  case "${flag}" in
    y) answer_yes_admin='y' ;;
    u) answer_yes_user='y' ;;
    *) print_usage
       exit 1 ;;
  esac
done

function ask
{
  while true; do
    echo -n '  (y|n) > '
    read s
    if [ "$s" == "y" ];then return 0; fi
    if [ "$s" == "n" ];then return 1; fi
  done
}
function ask_admin
{
  if [ "${answer_yes_admin}" == "y" ]; then return 0; fi
  if ask; then return 0; else return 1; fi
}
function ask_user
{
  if [ "${answer_yes_user}" == "y" ]; then return 0; fi
  if ask; then return 0; else return 1; fi
}

echo '[admin] Install Ubuntu core packages?'
if ask_admin; then
  # [Light desktop]
  sudo apt -y -f install xfce4 lightdm xfce4-power-manager xfce4-power-manager-plugins xfce4-goodies

  # [Useful packages]
  sudo apt -y -f install ibus ibus-mozc ibus-qt4 mozc-utils-gui
  sudo apt -y -f install openssh-server tcsh lv git tig htop dstat pcregrep nkf w3m xclip units g++ make cmake cmake-curses-gui automake libtool pkg-config gcc-doc glibc-doc kwrite kate konsole ffmpegthumbs kdegraphics-thumbnailers ark yakuake kdiff3 kompare nmap curl net-tools
  sudo apt -y -f install gnuplot
  sudo apt -y -f install aptitude apt-file

  # [Stop baloo (file indexing used by dolphin)]
  sudo apt remove baloo-kf5
fi


echo '[admin] Install ROS?'
if ask_admin; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  sudo apt update
  sudo apt -y -f install ros-melodic-desktop-full

  sudo apt -y -f install ros-melodic-moveit-commander ros-melodic-moveit-planners ros-melodic-moveit-plugins ros-melodic-moveit-ros ros-melodic-moveit-resources ros-melodic-cmake-modules ros-melodic-usb-cam  ros-melodic-rviz-visual-tools ros-melodic-code-coverage ros-melodic-joy ros-melodic-urdfdom-py ros-melodic-kdl-parser-py ros-melodic-code-coverage

  sudo apt -y -f install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

  sudo apt -y -f install python-pip

  sudo apt -y install python3-yaml python3-pip

  source /opt/ros/melodic/setup.bash
  sudo rosdep init
  rosdep update
fi

echo '[user] Install ROS?'
if ask_user; then
  source /opt/ros/melodic/setup.bash
  python -m pip install pybind11
  python3 -m pip install rospkg catkin_pkg
  rosdep update
fi

echo '[user] Configure .bashrc?'
if ask_user; then
  echo "$ros_bash_config" >> ~/.bashrc

  eval "$ros_bash_config"
fi

echo '[user] Setup workspace?'
if ask_user; then
  eval "$ros_bash_config"
  mkdir -p ~/ros_ws/ && cd ~/ros_ws/
  rosws init
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
  catkin_make
fi


echo '[admin] Setup for Dynamixel SDK?'
if ask_admin; then
  sudo usermod -a -G dialout $USER
fi

echo '[user] Install Dynamixel SDK?'
if ask_user; then
  mkdir -p ~/prg && cd ~/prg/
  git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
  cd DynamixelSDK/python/
  python setup.py install --user
fi

echo '[admin] Install OpenCV?'
if ask_admin; then
  sudo apt -y -f install libopencv-calib3d-dev libopencv-calib3d3.2 libopencv-contrib-dev libopencv-contrib3.2 libopencv-core-dev libopencv-core3.2 libopencv-dev libopencv-features2d-dev libopencv-features2d3.2 libopencv-flann-dev libopencv-flann3.2 libopencv-highgui-dev libopencv-highgui3.2 libopencv-imgcodecs-dev libopencv-imgcodecs3.2 libopencv-imgproc-dev libopencv-imgproc3.2 libopencv-ml-dev libopencv-ml3.2 libopencv-objdetect-dev libopencv-objdetect3.2 libopencv-photo-dev libopencv-photo3.2 libopencv-shape-dev libopencv-shape3.2 libopencv-stitching-dev libopencv-stitching3.2 libopencv-superres-dev libopencv-superres3.2 libopencv-ts-dev libopencv-video-dev libopencv-video3.2 libopencv-videoio-dev libopencv-videoio3.2 libopencv-videostab-dev libopencv-videostab3.2 libopencv-viz-dev libopencv-viz3.2 opencv-data opencv-doc python3-opencv
fi


echo '[user] Install MJPG-Streamer?'
if ask_user; then
  mkdir -p ~/prg/ && cd ~/prg/
  git clone https://github.com/akihikoy/mjpg-streamer.git mjpg-streamer2
  cd mjpg-streamer2/mjpg-streamer-experimental/
  make
fi


echo '[admin] Install libraries for AY-Tools & FingerVision?'
if ask_admin; then
  sudo apt -y -f install libboost-all-dev libboost-dev  python-setuptools python python-numpy python-scipy python-sklearn python-statsmodels python-pandas python-yaml  python-matplotlib python-tk  uvcdynctrl  python-rosinstall
  sudo apt -y -f install python-qt4 tmux rxvt-unicode-256color
fi

echo '[user] Install AY-Tools & FingerVision?'
if ask_user; then
  eval "$ros_bash_config"

  mkdir -p ~/ros_ws/ && cd ~/ros_ws/
  # FV+Gripper Kit:
  rosws merge https://raw.githubusercontent.com/akihikoy/ay_common/master/ay_ros/fv_gripper_kit.rosinstall
  rosws update

  rosmake ay_util_msgs
  rosmake ay_util
  rosmake fingervision

  # Create links to utility scripts
  cd ~
  ln -is ros_ws/ay_tools/ay_common/util/launcher/fv+gripper.sh .
  ln -is ros_ws/ay_tools/ay_common/util/misc/fv+update.sh .
  cp -ia ros_ws/ay_tools/ay_common/util/launcher/fv+config.sh .

  mkdir -p ~/data/data_gen/ ~/data/config/
  cp -ia `rospack find ay_fv_extra`/config/fvp_5_l.yaml ~/data/config/fvp300x_l.yaml
  cp -ia `rospack find ay_fv_extra`/config/fvp_5_r.yaml ~/data/config/fvp300x_r.yaml
  mkdir -p ~/.rviz/
  cp -ia `rospack find fv_gripper_ctrl`/config/default.rviz ~/.rviz/

  # BG image
  wget http://akihikoy.net/p/FVIncLogo/logo_blue.png -O ~/Downloads/logo_blue.png
fi

echo '[admin] Configure the system for AY-Tools & FingerVision?'
if ask_admin; then
  eval "$ros_bash_config"

  # Setup FV+ demo kit
  sudo ln -is `rospack find ay_util`/scripts/fix_usb_latency.sh /sbin/

  # For FV simulation:
  if [ ! -f /media/fvdata ];then
    sudo ln -is /home/$USER/ros_ws/ay_tools/fingervision/data /media/fvdata
  fi
fi


# Configuration:
echo '
===================================

Instruction: Add the following line to sudoers.

%dialout ALL = NOPASSWD: /sbin/fix_usb_latency.sh

'
echo '[admin] Run visudo?'
if ask; then
  sudo visudo
fi

echo '[user] Edit ~/fv+config.sh?'
if ask; then
  nano ~/fv+config.sh
fi


echo '
===================================

Instruction: Make symbolic links in /media/ pointing to the cameras.
e.g.
$ sudo ln -is /dev/v4l/by-path/pci-0000\:00\:14.0-usb-0\:7.1\:1.0-video-index0 /media/video_fv1
$ sudo ln -is /dev/v4l/by-path/pci-0000\:00\:14.0-usb-0\:1.1\:1.0-video-index0 /media/video_fv2

The camera ID can be listed by:
$ v4l2-ctl --list-devices
'
v4l2-ctl --list-devices

