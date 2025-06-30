#!/usr/bin/env python

import rospy
import csv
import os
from datetime import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates

class PoseLogger:
    def __init__(self):
        rospy.init_node('pose_logger', anonymous=True)

        # Get the file path from a ROS param, default to home directory
        self.log_file_path = rospy.get_param('~log_file', os.path.join(os.path.expanduser('~'), 'pose_log.csv'))
        
        # Subscribers
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

        # Data storage
        self.amcl_pose = None
        self.gazebo_pose = None
        self.robot_name = 'fetch' # The name of your robot model in Gazebo

        # Setup CSV file
        self.csv_file = open(self.log_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'amcl_x', 'amcl_y', 'ground_truth_x', 'ground_truth_y', 'error_x', 'error_y'])
        rospy.loginfo("Logging pose data to: %s", self.log_file_path)

        # Set a timer to log data at a regular interval (e.g., every 1 second)
        rospy.Timer(rospy.Duration(1.0), self.log_data_callback)

    def amcl_callback(self, msg):
        """Callback to store the latest AMCL pose estimate."""
        self.amcl_pose = msg.pose.pose

    def gazebo_callback(self, msg):
        """Callback to store the latest Gazebo ground truth pose."""
        try:
            # Find the index of the robot in the model states message
            robot_index = msg.name.index(self.robot_name)
            self.gazebo_pose = msg.pose[robot_index]
        except ValueError:
            # Robot not found in the message, which can happen occasionally
            self.gazebo_pose = None
            rospy.logwarn_throttle(5, "Could not find '%s' in /gazebo/model_states", self.robot_name)

    def log_data_callback(self, event):
        """Timer callback that writes the stored data to the CSV file."""
        if self.amcl_pose and self.gazebo_pose:
            # Get current ROS time for the timestamp
            now = rospy.Time.now()

            # Calculate error
            error_x = self.amcl_pose.position.x - self.gazebo_pose.position.x
            error_y = self.amcl_pose.position.y - self.gazebo_pose.position.y

            # Write data to CSV
            self.csv_writer.writerow([
                now.to_sec(),
                self.amcl_pose.position.x,
                self.amcl_pose.position.y,
                self.gazebo_pose.position.x,
                self.gazebo_pose.position.y,
                error_x,
                error_y
            ])

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        """Close the CSV file cleanly when the node is shut down."""
        if self.csv_file:
            self.csv_file.close()
            rospy.loginfo("CSV file closed.")

if __name__ == '__main__':
    try:
        logger = PoseLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass

