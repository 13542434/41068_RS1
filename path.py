#!/usr/bin/env python2.7
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Initialize the ROS node
rospy.init_node('send_goal')

# Create an action client connected to the move_base action server
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# Wait for the action server to become available
client.wait_for_server()

# Define the goal position and orientation (quaternion)
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'  # The frame in which the goal position is defined
goal_pose.pose.position.x = 1.5  # X-coordinate of the goal position (meters)
goal_pose.pose.position.y = -2.0  # Y-coordinate of the goal position (meters)
goal_pose.pose.orientation.z = 0.0  # Quaternion representation of the goal orientation
goal_pose.pose.orientation.w = 1.0

# Create a MoveBaseGoal object and set the target pose
goal = MoveBaseGoal()
goal.target_pose = goal_pose

# Send the goal to the action server and wait for the result
client.send_goal(goal)
client.wait_for_result()

# Print the result of the navigation
if client.get_state() == 3:
    rospy.loginfo("Goal reached!")
else:
    rospy.logwarn("Failed to reach the goal!")
