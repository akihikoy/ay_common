ay_common
==================
Common stuff of ay_tools.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Usage
==================

ay_ros/ay_tool.rosinstall
---------------------------
Use this file with rosws.

## Initialization

```
$ mkdir ~/ros_ws/
$ cd ~/ros_ws/
ros_ws$ rosws init
ros_ws$ rosws merge https://raw.githubusercontent.com/akihikoy/ay_common/master/ay_ros/ay_tool.rosinstall
ros_ws$ rosws update
```

## Update
```
ros_ws$ rosws update
```

ay_ros/ay_tool_prv.rosinstall
--------------------------------------------
You must have a permission to access the private repositories on GitHub.

Usage: Replace `ay_tool.rosinstall` by `ay_tool_prv.rosinstall`.


Troubles
==================
Send e-mails to the author.
