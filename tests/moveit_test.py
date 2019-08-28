#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 14 12:47:25 2018

@author: chris
"""

import rospy
import moveit_commander
import moveit_msgs.msg
import tf

import geometry_msgs.msg

class moveRobot:
    
    def __init__(self, move_group="harvey_pi_camera"):

        self.group = moveit_commander.MoveGroupCommander(move_group)
        self.planning_frame = self.group.get_planning_frame()
        self.tf_listener = tf.TransformListener()
        
    def moveToPose(self,poseStamped)
    
        if not self.planning_frame == poseStamped.header.frame_id:
             self.tf_listener.waitForTransform(poseStamped.header.frame_id, self.planning_frame, rospy.Time(), rospy.Duration(0.5))
             
             transformed_pose = self.tf_listener.transformPose(self.planning_frame,poseStamped.pose)
             
             poseStamped.pose = transformed_pose
             
        group.set_pose_target(poseStamped.pose )
             
        print "planning movement"
        plan1 = group.plan()
        
        print "Executing movement"
        group.go(wait=True)
        


if __name__=="__main__":
    

    interface = "ROS"

    if interface == "ROS":
        rospy.init_node("harvey_move_to_see")
        print "Running move to see node"

        mvs = move_to_see(9,"ROS",step_size=0.001, size_weight=1.0, manip_weight=0.0,end_tolerance=0.75,max_pixel=0.4)
        mvs.initCameraPosition()

    else:
        mvs = move_to_see(9,"VREP")

    data = mvs.execute()