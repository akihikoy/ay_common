#!/bin/bash
#\file    fv+gripper-setup.sh
#\brief   Script to setup a Linux PC (Ubuntu 18.04) for FV+GripperKit.
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

echo 'Install Ubuntu core packages?'
if ask; then
  # [Light desktop]
  sudo apt -f install xfce4 lightdm

  # [Useful packages]
  sudo apt -f install ibus ibus-mozc ibus-qt4 mozc-utils-gui
  sudo apt -f install openssh-server tcsh lv git tig htop dstat pcregrep nkf w3m xclip units g++ make cmake cmake-curses-gui automake libtool pkg-config gcc-doc glibc-doc kwrite kate konsole ffmpegthumbs kdegraphics-thumbnailers ark yakuake kdiff3 kompare nmap curl net-tools
  sudo apt -f install gnuplot
  sudo apt -f install aptitude apt-file

  # [Stop baloo (file indexing used by dolphin)]
  sudo apt remove baloo-kf5
fi


echo 'Install ROS?'
if ask; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  sudo apt update
  sudo apt install ros-melodic-desktop-full

  sudo apt -f install ros-melodic-moveit-commander ros-melodic-moveit-planners ros-melodic-moveit-plugins ros-melodic-moveit-ros ros-melodic-moveit-resources ros-melodic-cmake-modules ros-melodic-usb-cam  ros-melodic-rviz-visual-tools ros-melodic-code-coverage ros-melodic-joy ros-melodic-urdfdom-py ros-melodic-kdl-parser-py ros-melodic-code-coverage

  sudo apt-get -f install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

  sudo apt-get -f install python-pip
  python -m pip install pybind11

  sudo apt-get install python3-yaml python3-pip
  python3 -m pip install rospkg catkin_pkg

  . /opt/ros/melodic/setup.bash
  sudo rosdep init
  rosdep update
fi

echo 'Configure .bashrc?'
if ask; then
  echo "
#ROS Configuration
#ROS_DISTR=kinetic
ROS_DISTR=melodic
. /opt/ros/\$ROS_DISTR/setup.bash
. ~/catkin_ws/devel/setup.bash
. /opt/ros/\$ROS_DISTR/share/rosbash/rosbash

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:\$HOME/ros_ws:\${HOME}/prg/ay_test/ros
" >> ~/.bashrc

  . .bashrc
fi

echo 'Setup workspace?'
if ask; then
  mkdir -p ~/ros_ws/ && cd ~/ros_ws/
  rosws init
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
  catkin_make
fi


echo 'Install Dynamixel SDK?'
if ask; then
  sudo usermod -a -G dialout $USER

  mkdir -p ~/prg && cd ~/prg/
  git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
  cd DynamixelSDK/python/
  python setup.py install --user
fi

echo 'Install OpenCV?'
if ask; then
  sudo apt-get -f install libopencv-calib3d-dev libopencv-calib3d3.2 libopencv-contrib-dev libopencv-contrib3.2 libopencv-core-dev libopencv-core3.2 libopencv-dev libopencv-features2d-dev libopencv-features2d3.2 libopencv-flann-dev libopencv-flann3.2 libopencv-highgui-dev libopencv-highgui3.2 libopencv-imgcodecs-dev libopencv-imgcodecs3.2 libopencv-imgproc-dev libopencv-imgproc3.2 libopencv-ml-dev libopencv-ml3.2 libopencv-objdetect-dev libopencv-objdetect3.2 libopencv-photo-dev libopencv-photo3.2 libopencv-shape-dev libopencv-shape3.2 libopencv-stitching-dev libopencv-stitching3.2 libopencv-superres-dev libopencv-superres3.2 libopencv-ts-dev libopencv-video-dev libopencv-video3.2 libopencv-videoio-dev libopencv-videoio3.2 libopencv-videostab-dev libopencv-videostab3.2 libopencv-viz-dev libopencv-viz3.2 opencv-data opencv-doc python3-opencv
fi


echo 'Install MJPG-Streamer?'
if ask; then
  mkdir -p ~/prg/ && cd ~/prg/
  git clone https://github.com/akihikoy/mjpg-streamer.git mjpg-streamer2
  cd mjpg-streamer2/mjpg-streamer-experimental/
  make
fi

echo 'Install AY-Tools & FingerVision?'
if ask; then
  mkdir -p ~/ros_ws/ && cd ~/ros_ws/
  # FV+Gripper Kit:
  rosws merge https://raw.githubusercontent.com/akihikoy/ay_common/master/ay_ros/fv_gripper_kit.rosinstall

  rosmake ay_util fingervision

  # Setup FV+ demo kit
  sudo ln -s `rospack find ay_util`/scripts/fix_usb_latency.sh /sbin/

  # FV+Gripper Kit
  cd ~
  ln -s ros_ws/ay_tools/ay_common/util/launcher/fv+gripper.sh .

  # Common config:
  cd ~
  cp ros_ws/ay_tools/ay_common/util/launcher/fv+config.sh .

  # For FV simulation:
  sudo ln -s /home/$USER/ros_ws/ay_tools/fingervision/data /media/fvdata

  mkdir -p ~/data/data_gen/ ~/data/config/
  cp -a `rospack find ay_fv_extra`/config/fvp_5_l.yaml ~/data/config/fvp300x_l.yaml
  cp -a `rospack find ay_fv_extra`/config/fvp_5_r.yaml ~/data/config/fvp300x_r.yaml
  mkdir -p ~/.rviz/
  cp -a `rospack find fv_gripper_ctrl`/config/default.rviz ~/.rviz/
fi

# BG image
cd ~/Downloads
wget https://akihikoy.net/p/FVIncLogo/logo_blue.png


# Configuration:
echo '
%dialout ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh

Add the above line to sudoers.
'
sudo visudo

echo 'Edit ~/fv+config.sh'
nano ~/fv+config.sh

v4l2-ctl --list-devices
echo 'Make symbolic links in /media/ pointing to those cameras.
e.g.
sudo ln -is /dev/v4l/by-path/pci-0000\:00\:14.0-usb-0\:7.1\:1.0-video-index0 /media/video_fv1
sudo ln -is /dev/v4l/by-path/pci-0000\:00\:14.0-usb-0\:1.1\:1.0-video-index0 /media/video_fv2
'

