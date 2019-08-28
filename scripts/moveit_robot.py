#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 14 12:47:25 2018

@author: chris
"""

import rospy
import moveit_commander
import moveit_msgs.msg
#import tf
import tf2_ros

import geometry_msgs.msg
import tf2_geometry_msgs

class moveRobot:
    
    def __init__(self, move_group="harvey_pi_camera"):

        self.group = moveit_commander.MoveGroupCommander(move_group)
        self.planning_frame = self.group.get_planning_frame()
                
        #self.tf_listener = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
    def moveToPose(self,poseStamped):
    
        if not (self.planning_frame == poseStamped.header.frame_id):
            
             print "Looking up transform from planning frame: ", poseStamped.header.frame_id
             print "to planning frame: ", self.planning_frame
             #self.tfBuffer.waitForTransform(poseStamped.header.frame_id, self.planning_frame, rospy.Time(), rospy.Duration(5))
             
             try:
                 trans = self.tfBuffer.lookup_transform(self.planning_frame[1:], poseStamped.header.frame_id[1:], rospy.Time(0), rospy.Duration(1.0))
                  #spose = gmsg.PoseStamped()
                  #spose.pose = pose
                  #spose.header.stamp = rospy.Time().now
                  #spose.header.frame_id = from_frame
                 transformed_pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans)

                 
             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
                     print(e)
                     rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
                     return None
             
             
             
             #transformed_pose = self.tf_listener.transformPose(self.planning_frame, poseStamped)
             
             poseStamped.pose = transformed_pose
             
        self.group.set_pose_target(poseStamped.pose )
             
        print "planning movement"
        plan1 = self.group.plan()
        
        print "Executing movement"
        self. group.go(wait=True)
        


if __name__=="__main__":
    

    rospy.init_node("moveit_robot_test")
    print "Running moveit robot node"
    
    robot = moveRobot()
    
    pose_target_stamped = geometry_msgs.msg.PoseStamped()
    pose_target_stamped.header.frame_id = "/pi_camera_link"
    pose_target_stamped.header.stamp = rospy.Time.now()
    
    pose_target_stamped.pose.orientation.w = 1.0
    pose_target_stamped.pose.position.x = 0.0
    pose_target_stamped.pose.position.y = 0.0
    pose_target_stamped.pose.position.z = 0.0

    robot.moveToPose(pose_target_stamped)