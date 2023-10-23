import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math

class position:
    x = 0.0 # float
    y = 0.0# float
    z = 0.0# float
    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        
class hazard_type:
    NONE    = 'None'
    TEMP    = 'Temperature'
    C02     = 'Carbon Dioxide'
    AMMONIA = 'Ammonia'
    NOISE   = 'Noise'

class Hazard:
    hazard = hazard_type.NONE # hazard_type
    pos = (0.0,0.0,0.0) # position
    value = 0.0 # float
    
    def __init__(self, hazard = hazard_type.NONE, pos = (0.0,0.0,0.0), value = 0.0):
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
    
    def odom_callback(self, data):
        self.current_position = data.pose.pose.position

    def calculate_distance(self, point): # returns float
        return math.sqrt((point.x - self.current_position.x)**2 + (point.y - self.current_position.y)**2)
    
    def check_hazard(self): # returns list
        if not self.current_position:
            return None
        
        result = []
        for hazard in self.hazards:
            distance = self.calculate_distance(hazard.pos)
            if distance <= self.distance_threshold:
                result.append((hazard.hazard_type,hazard.value))
        return result
    
    def init_ros(self):
        pass
        rospy.init_node('distance_to_hazards')
        # Subscribe to the /odom topic to get the robot's current position
        rospy.Subscriber("/odom", Odometry, self.odom_callback)     
    
    def __init__(self):
        self.init_ros()
        pass
        
    def monitor_hazards(self):
        # This loop will run until the node is stopped
        while not rospy.is_shutdown():
            detected_hazards = self.check_hazard()
            if detected_hazards and len(detected_hazards) > 0:
                print("Hazards Detected at (" + str(self.current_position.x) + "," + str(self.current_position.y) + "):")
                for type,value in detected_hazards:
                    print(str(type) + "\t" + str(value))
            else:
                print("No hazards detected at current position")
            rospy.sleep(1)  # Check every second. You can modify this rate as needed.

if __name__ == "__main__":
    hazards = Hazards_Controller() 
    x, y, z = [float(i) for i in raw_input("Initial Position \"x,y,z\" >>> ").split(',')]
    hazards.current_position = position((x, y, z))
    if not hazards.current_position:
        print("Failed to get the robot's initial position!")
        exit(1)
    hazards.monitor_hazards()  # Start monitoring hazards

        
# print(f"Distance to heat_point ({heat_point['x']}, {heat_point['y']}) is: {distance:.2f} meters")
        
