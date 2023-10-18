# import rospy
# import actionlib
# from geometry_msgs.msg import PoseStamped
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from nav_msgs.msg import Odometry
import math
from enum import Enum

class position:
    x : float
    y : float
    z : float
    def __init__(self, pos : tuple[float,float,float]) -> None:
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
class hazard_type(str, Enum):
    NONE    = 'None'
    TEMP    = 'Temperature'
    C02     = 'Carbon Dioxide'
    AMMONIA = 'Ammonia'
    NOISE   = 'Noise'

class Hazard:
    hazard : hazard_type
    pos : position
    value : float
    
    def __init__(self, hazard : hazard_type = hazard_type.NONE, pos : tuple[float,float,float] = (0.0,0.0,0.0), value : float = 0.0) -> None:
        self.hazard_type = hazard
        self.pos = position(pos)
        self.value = value
    
            

class Hazards_Controller:
    # Global variable to store the robot's current position
    current_position = None
    distance_threshold = 1.0 # Distance between robot and hazard to call hazard
    hazards = [
        Hazard(
            hazard=hazard_type.TEMP,
            pos=(4.0,-2.0,0.0),
            value=50.0
        ),
        Hazard(
            hazard=hazard_type.C02,
            pos=(3.0,-2.0,0.0),
            value=4.2
        ),
        Hazard(
            hazard=hazard_type.NOISE,
            pos=(3.0,-1.0,0.0),
            value=105.2
        ),
    ]
    
    def odom_callback(self, data) -> None:
        self.current_position = data.pose.pose.position

    def calculate_distance(self, point : position) -> float:
        return math.sqrt((point.x - self.current_position.x)**2 + (point.y - self.current_position.y)**2)
    
    def check_hazard(self) -> list:
        result : list = []
        for hazard in self.hazards:
            distance = self.calculate_distance(hazard.pos)
            if distance <= self.distance_threshold:
                result.append((hazard.hazard_type,hazard.value))
        return result
    
    def init_ros() -> None:
        rospy.init_node('distance_to_heat_point')

        # Subscribe to the /odom topic to get the robot's current position
        rospy.Subscriber("/odom", Odometry, self.odom_callback)     
    
    def __init__(self) -> None:
        # self.init_ros()
        pass




if __name__ == "__main__":
    # Use this as a test function
    hazards = Hazards_Controller() 
    # Wait for a moment to ensure we get the robot's position
    # rospy.sleep(1)
    
    x,y,z = [float(i) for i in input("Current Position \"x,y,z\" >>> ").split(',')]
    hazards.current_position = position((x,y,z))

    if not hazards.current_position:
        print("Failed to get the robot's current position!")
        exit(1)

    detected_hazards = hazards.check_hazard()
    
    if len(detected_hazards) > 0:
        print(f"Hazards Detected at ({hazards.current_position.x},{hazards.current_position.y}):")
        for type,value in detected_hazards:
            print(f"{type}:\t{value}")
    else:
        print("No hazards detected at current position")
        
# print(f"Distance to heat_point ({heat_point['x']}, {heat_point['y']}) is: {distance:.2f} meters")
        
