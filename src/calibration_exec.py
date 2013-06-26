#! /usr/bin/env python

import roslib; roslib.load_manifest('velma_calibration')
import rospy

# Brings in the SimpleActionClient
import actionlib

import control_msgs.msg
from velma_calibration.srv import *

def fibonacci_client():

    rospy.wait_for_service('/collect_data/capture')
    capture = rospy.ServiceProxy('/collect_data/capture', Capture)
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/head_controller/point_head_action', control_msgs.msg.PointHeadAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.PointHeadGoal()
    
    goal.target.header.frame_id = 'right_tool_link'
    goal.target.point.x = 0.15;
    goal.target.point.y = 0;
    goal.target.point.z = 0.3;
    
    goal.pointing_frame = "stereo_left_link";
    goal.min_duration = rospy.Duration (1.5);
    goal.max_velocity = 1.0;
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.sleep(1.5)
    capture()

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.PointHeadGoal()
    
    goal.target.header.frame_id = 'right_tool_link'
    goal.target.point.x = 0.15;
    goal.target.point.y = 0;
    goal.target.point.z = 0.3;
    
    goal.pointing_frame = "kinect_left_rgb_link";
    goal.min_duration = rospy.Duration (1.5);
    goal.max_velocity = 1.0;
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.sleep(1.5)
    capture()
    
    # Creates a goal to send to the action server.
    goal = control_msgs.msg.PointHeadGoal()
    
    goal.target.header.frame_id = 'right_tool_link'
    goal.target.point.x = 0.15;
    goal.target.point.y = 0;
    goal.target.point.z = 0.3;
    
    goal.pointing_frame = "kinect_right_rgb_link";
    goal.min_duration = rospy.Duration (1.5);
    goal.max_velocity = 1.0;
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.sleep(1.5)
    capture()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        fibonacci_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
