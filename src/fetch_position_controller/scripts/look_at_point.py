#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped

def point_head(x, y, z):
    rospy.init_node('head_pointing_demo')

    client = actionlib.SimpleActionClient('/head_controller/point_head', PointHeadAction)
    rospy.loginfo("Waiting for head controller...")
    client.wait_for_server()

    goal = PointHeadGoal()
    goal.target = PointStamped()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z

    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 1.0
    goal.pointing_frame = "head_pan_link"
    goal.pointing_axis.x = 1.0
    goal.pointing_axis.y = 0.0
    goal.pointing_axis.z = 0.0

    rospy.loginfo("Sending head point goal to ({}, {}, {})".format(x, y, z))
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Head movement complete.")

if __name__ == '__main__':
    try:
        x = float(input("Enter x (in base_link): "))
        y = float(input("Enter y (in base_link): "))
        z = float(input("Enter z (in base_link): "))
        point_head(x, y, z)
    except rospy.ROSInterruptException:
        pass

