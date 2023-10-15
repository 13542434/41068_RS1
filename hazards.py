import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math


class Hazards:
    # Global variable to store the robot's current position
    current_position = None
    distance_threshold = 1.0 # Distance between robot and hazard to call hazard
    heat_points = [
        {
            'x': 4.0,
            'y': -2.0,
            'z': 0.0,
            'temp': 50.0,
        }
    ]
    
    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

    def calculate_distance(self, point) -> float:
        return math.sqrt((point['x'] - self.current_position.x)**2 + (point['y'] - self.current_position.y)**2)
    
    def check_hazard(self) -> list:
        result : list = []
        for heat_point in hazards.heat_points:
            distance = self.calculate_distance(heat_point)
            if distance < self.distance_threshold:
                result.append(heat_point['temp'])
        return result
                
    
    def __init__(self) -> None:
        rospy.init_node('distance_to_heat_point')

        # Subscribe to the /odom topic to get the robot's current position
        rospy.Subscriber("/odom", Odometry, self.odom_callback)




if __name__ == "__main__":
    # Use this as a test function
    hazards = Hazards() 
    # Wait for a moment to ensure we get the robot's position
    rospy.sleep(1)

    if not hazards.current_position:
        print("Failed to get the robot's current position!")
        exit(1)

    detected_hazards = hazards.check_hazard()
    
    if len(detected_hazards) > 0:
        for temperature in detected_hazards:
            print(f"Hazard detected at {hazards.current_position.x},{hazards.current_position.y} of {temperature}C")
    else:
        print("No hazards detected at current position")
        
# print(f"Distance to heat_point ({heat_point['x']}, {heat_point['y']}) is: {distance:.2f} meters")
        
