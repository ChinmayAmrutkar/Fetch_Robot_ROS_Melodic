#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pow
import tf

class PositionController:
    def __init__(self):
        rospy.init_node('go_to_point_controller')
        
        self.pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)

        self.pose = None  # Will hold robot's current position

        rospy.loginfo("Waiting for odom...")
        while self.pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        x_goal = float(input("Enter goal x: "))
        y_goal = float(input("Enter goal y: "))

        self.move_to_goal(x_goal, y_goal)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
    
    def move_to_goal(self, x_goal, y_goal):
        vel = Twist()

        while not rospy.is_shutdown():
            # Current Position
            x = self.pose.position.x
            y = self.pose.position.y

            # Compute distance to goal
            distance = sqrt(pow((x_goal - x), 2) + pow((y_goal - y), 2))

            # Break when close enough
            if distance < 0.05:
                rospy.loginfo("Goal reached!")
                break

            # Angle to goal
            angle_to_goal = atan2(y_goal - y, x_goal - x)

            # Compute error
            angle_error = angle_to_goal - self.yaw

            # Control law
            vel.linear.x = min(0.3, distance)
            vel.angular.z = 1.0 * angle_error

            self.pub.publish(vel)
            self.rate.sleep()

        # Stop the robot
        vel.linear.x = 0
        vel.angular.z = 0
        self.pub.publish(vel)

if __name__ == '__main__':
    try:
        PositionController()
    except rospy.ROSInterruptException:
        pass

