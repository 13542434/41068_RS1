import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math

# Global variable to store the robot's current position
current_position = None

def odom_callback(data):
    global current_position
    current_position = data.pose.pose.position

def calculate_distance(point1, point2):
    return math.sqrt((point2['x'] - point1.x)**2 + (point2['y'] - point1.y)**2)

if __name__ == "__main__":
    rospy.init_node('distance_to_heat_point')

    # Subscribe to the /odom topic to get the robot's current position
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Wait for a moment to ensure we get the robot's position
    rospy.sleep(1)

    heat_points = [
        {
            'x': 4.0,
            'y': -2.0,
            'z': 0.0,
        }
    ]

    if current_position:
        for heat_point in heat_points:
            distance = calculate_distance(current_position, heat_point)
            print(f"Distance to heat_point ({heat_point['x']}, {heat_point['y']}) is: {distance:.2f} meters")
    else:
        print("Failed to get the robot's current position!")
