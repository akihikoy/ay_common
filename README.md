ay_common
==================
Common stuff of ay_tools.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Usage
==================

ay_ros/ay_tools.rosinstall
---------------------------
Use this file with rosws.

You need to install the `python-rosinstall` package.

```
$ sudo apt-get -f install python-rosinstall
```

## Initialization

```
$ mkdir ~/ros_ws/
$ cd ~/ros_ws/
ros_ws$ rosws init
ros_ws$ rosws merge https://raw.githubusercontent.com/akihikoy/ay_common/master/ay_ros/ay_tools.rosinstall
ros_ws$ rosws update
```

## Update
```
ros_ws$ rosws update
```

ay_ros/ay_tools_prv.rosinstall
--------------------------------------------
You must have a permission to access the private repositories on GitHub.

Usage: Replace `ay_tools.rosinstall` by `ay_tools_prv.rosinstall`.


Troubles
==================
Send e-mails to the author.
