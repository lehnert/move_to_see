#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 14 21:51:29 2018

@author: chris
"""

import rospy
import ros_interface_naive
rospy.init_node("harvey_move_to_see")

interface = ros_interface_naive.ros_interface(9) 

pose = interface.getCapsicumDepth()

#_,_,camera_poses = interface.getCameraPositions()

print "Current poses: ", pose

#m = interface.getManipulability(camera_poses)