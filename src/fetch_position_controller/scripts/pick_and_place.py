#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import tf
import actionlib
import moveit_commander
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pow
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class PositionController:
    def __init__(self):
        self.pose = None
        self.yaw = 0.0
        self.vel_pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("Waiting for odometry data...")
        while self.pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def move_base(self, goal_x, goal_y):
        """Moves the robot base to a given (x, y) goal using proportional control"""
        vel = Twist()
        while not rospy.is_shutdown():
            x = self.pose.position.x
            y = self.pose.position.y
            distance = sqrt((goal_x - x)**2 + (goal_y - y)**2)
            if distance < 0.05:
                break
            angle_to_goal = atan2(goal_y - y, goal_x - x)
            angle_error = angle_to_goal - self.yaw
            vel.linear.x = min(0.3, distance)
            vel.angular.z = 0.8 * angle_error
            self.vel_pub.publish(vel)
            self.rate.sleep()
        self.vel_pub.publish(Twist())  # stop


def move_arm_to(x, y, z, roll, pitch, yaw):
    """Moves the Fetch robot arm to the specified Cartesian pose"""
    group = moveit_commander.MoveGroupCommander("arm")
    q = quaternion_from_euler(roll, pitch, yaw)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    group.set_pose_target(pose)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.loginfo("Arm movement: %s", "Success" if success else "Failed")


def control_gripper(position=0.04, effort=50.0):
    """Sends a command to open or close the gripper"""
    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    client.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = effort
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Gripper moved to position %.3f", position)

if __name__ == '__main__':
    rospy.init_node("fetch_pick_and_place_final")
    moveit_commander.roscpp_initialize(sys.argv)

    # Step 1: Move the base near the cylinder
    base = PositionController()
    rospy.loginfo("Moving base to pick location...")
    base.move_base(1.0, 0.0)

    # Step 2: Move arm to pre-grasp hover pose
    rospy.sleep(1.0)
    rospy.loginfo("Moving arm to hover pose above object...")
    move_arm_to(0.5, 0.0, 0.75, 0.0, 1.57, 0.0)

    # Step 3: Move arm closer to object
    rospy.sleep(1.0)
    rospy.loginfo("Approaching object...")
    move_arm_to(0.65, 0.0, 0.75, 0.0, 1.57, 0.0)

    # Step 4: Move arm down to grasp height
    rospy.sleep(1.0)
    rospy.loginfo("Lowering to grasp height...")
    move_arm_to(0.65, 0.0, 0.65, 0.0, 1.57, 0.0)

    # Step 5: Close gripper to grasp the object
    rospy.sleep(1.0)
    rospy.loginfo("Grasping object...")
    control_gripper(position=0.0)

    # Step 6: Lift the object
    rospy.loginfo("Lifting object...")
    move_arm_to(0.6, 0.0, 0.85, 0.0, 1.57, 0.0)

    # Step 7: Move base back to drop location
    rospy.loginfo("Moving base to place location...")
    base.move_base(1.0, 0.0)

    # Step 8: Lower arm to place height
    rospy.sleep(1.0)
    rospy.loginfo("Lowering object for placement...")
    move_arm_to(0.5, 0.0, 0.7, 0.0, 1.57, 0.0)

    # Step 9: Open gripper to release object
    rospy.loginfo("Releasing object...")
    control_gripper(position=0.04)

    rospy.loginfo("✅ Pick and place task completed.")

