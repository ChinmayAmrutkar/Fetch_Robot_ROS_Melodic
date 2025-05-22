#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def move_to_pose(x, y, z, roll, pitch, yaw):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_fetch_arm_6dof', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    group.set_planning_time(5.0)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.1)

    rospy.loginfo("Planning to move to pose: x=%.3f, y=%.3f, z=%.3f, r=%.2f, p=%.2f, y=%.2f (rad)", x, y, z, roll, pitch, yaw)

    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = qx
    pose_target.orientation.y = qy
    pose_target.orientation.z = qz
    pose_target.orientation.w = qw

    group.set_pose_target(pose_target)
    plan = group.plan()

    if plan and hasattr(plan, 'joint_trajectory') and len(plan.joint_trajectory.points) > 0:
        rospy.loginfo("Plan successful! Executing...")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
    else:
        rospy.logwarn("Failed to plan to target pose.")

    moveit_commander.roscpp_shutdown()

def control_gripper(position, effort):
    rospy.loginfo("Sending gripper command: position=%.3f, effort=%.1f", position, effort)
    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    client.wait_for_server()

    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = effort
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Gripper command completed.")

if __name__ == '__main__':
    try:
        choice = raw_input(
            "\nChoose mode:\n"
            "[1] Go to Cartesian pose\n"
            "[2] Go to tucked (home) joint pose\n"
            "[3] Control gripper (open/close)\n"
            "Enter option: "
        ).strip()

        if choice == '1':
            print("Enter target pose:")
            x = float(input("  x (e.g. 0.6): "))
            y = float(input("  y (e.g. 0.0): "))
            z = float(input("  z (e.g. 0.6): "))
            roll  = float(input("  roll  (in radians, e.g. 0): "))
            pitch = float(input("  pitch (in radians, e.g. 1.57): "))
            yaw   = float(input("  yaw   (in radians, e.g. 0): "))
            move_to_pose(x, y, z, roll, pitch, yaw)

        elif choice == '2':
            rospy.init_node('moveit_fetch_tucked_pose', anonymous=True)
            moveit_commander.roscpp_initialize(sys.argv)
            group = moveit_commander.MoveGroupCommander("arm")

            rospy.loginfo("Planning to safe tucked pose...")

            tucked_joint_pose = {
                'shoulder_pan_joint':  1.32,
                'shoulder_lift_joint': 1.40,
                'upperarm_roll_joint': -0.20,
                'elbow_flex_joint':    1.72,
                'forearm_roll_joint':  0.0,
                'wrist_flex_joint':    1.66,
                'wrist_roll_joint':    0.0
            }

            group.set_joint_value_target(tucked_joint_pose)
            group.set_max_velocity_scaling_factor(0.2)
            group.set_max_acceleration_scaling_factor(0.1)

            plan = group.plan()
            if plan and len(plan.joint_trajectory.points) > 0:
                rospy.loginfo("Executing tucked arm position...")
                group.go(wait=True)
                group.stop()
            else:
                rospy.logwarn("Failed to plan to tucked pose.")

            moveit_commander.roscpp_shutdown()

        elif choice == '3':
            rospy.init_node('fetch_gripper_control', anonymous=True)
            action = raw_input("Type 'open' or 'close': ").strip().lower()
            if action == 'open':
                control_gripper(position=0.04, effort=50.0)
            elif action == 'close':
                control_gripper(position=0.0, effort=50.0)
            else:
                rospy.logwarn("Invalid gripper command.")

        else:
            print("Invalid option. Exiting.")

    except rospy.ROSInterruptException:
        pass

