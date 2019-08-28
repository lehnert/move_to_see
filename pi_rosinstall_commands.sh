#!/bin/bash

#grab other ros packages and generate ros install file
rosinstall_generator ros_comm message_generation std_msgs geometry_msgs sensor_msgs actionlib_msgs --rosdistro kinetic --deps --wet-only --exclude collada_parser collada_urdf --tar > kinetic-custom_ros.rosinstall

# merge ros install files
wstool merge -t src kinetic-custom_ros.rosinstall

#build ros src FILES_STRING
 sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
