#!/bin/bash
source ~/harvey_ws/devel/setup.bash
export ROS_MASTER_URI="http://192.168.1.100:11311"
export ROS_IP=$(hostname -I)
exec "$@"
