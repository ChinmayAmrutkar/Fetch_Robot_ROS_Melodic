#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# List of Fetch's 7 arm joints in order
joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'upperarm_roll_joint',
    'elbow_flex_joint',
    'forearm_roll_joint',
    'wrist_flex_joint',
    'wrist_roll_joint'
]

# Define a dictionary of named poses
named_poses = {
    'ready': [0.0, 0.4, 0.0, 1.4, 0.0, 1.2, 0.0],
    'stow': [0.0, 1.2, 0.0, 1.8, 0.0, 1.5, 0.0],
    'pickup': [0.3, 0.6, 0.0, 1.2, 0.0, 1.1, 0.0]
}

def move_arm(joint_positions):
    rospy.init_node('move_fetch_arm')

    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )

    rospy.loginfo("Waiting for arm controller action server...")
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(3.0)
    goal.trajectory.points.append(point)

    rospy.loginfo("Sending goal to arm...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Arm reached target position.")

if __name__ == '__main__':
    try:
        print("\nAvailable poses: {}".format(", ".join(named_poses.keys())))
        pose_name = raw_input("Enter pose name: ").strip().lower()
        if pose_name not in named_poses:
            rospy.logwarn("Invalid pose name. Using 'ready'.")
            pose_name = 'ready'
        move_arm(named_poses[pose_name])
    except rospy.ROSInterruptException:
        pass

