import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
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
    pub_detected = None
    pub_groundtruth = None
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
    
    _rate = None
    _pointcloud = None
    _groundtruth = None
    
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
    
    def generate_groundtruth(self):
        self._groundtruth = PointCloud()
        self._groundtruth.header = Header()
        self._groundtruth.header.frame_id = 'map'
        self._groundtruth.header.stamp = rospy.Time.now()
        for hazard in self.hazards:
            self._groundtruth.points.append(Point32(
                hazard.pos.x,
                hazard.pos.y,
                hazard.pos.z,
            ))
    
    def init_ros(self):
        pass
        rospy.init_node('distance_to_hazards')
        self._rate = rospy.Rate(5)
        # Subscribe to the /odom topic to get the robot's current position
        rospy.Subscriber("/odom", Odometry, self.odom_callback)     
        self.pub_detected = rospy.Publisher('/hazards/detected', PointCloud, queue_size=10)
        self.pub_groundtruth = rospy.Publisher('/hazards/groundtruth', PointCloud, queue_size=10)
        
        self._pointcloud = PointCloud()
        self._pointcloud.header = Header()
        self._pointcloud.header.frame_id = 'map'
        self._pointcloud.header.stamp = rospy.Time.now()
        
        self.generate_groundtruth()
    
    def __init__(self):
        self.init_ros()
        pass

    def run(self):
        while not rospy.is_shutdown():
            detected_hazards = self.check_hazard()
            if detected_hazards and len(detected_hazards) > 0:
                print("Hazards Detected at (" + str(self.current_position.x) + "," + str(self.current_position.y) + "):")
                self._pointcloud.header.stamp = rospy.Time.now()
                
                for type,value in detected_hazards:
                    self._pointcloud.points.append(Point32(self.current_position.x,self.current_position.y,self.current_position.z))
                    print(str(type) + "\t" + str(value))
            else:
                print("No hazards detected at current position")
            self.pub_detected.publish(self._pointcloud)
            self.pub_groundtruth.publish(self._groundtruth)
            self._rate.sleep()
      
def test():
    hazards = Hazards_Controller() 
    x, y, z = [float(i) for i in raw_input("Initial Position \"x,y,z\" >>> ").split(',')]
    hazards.current_position = position((x, y, z))
    if not hazards.current_position:
        print("Failed to get the robot's initial position!")
        exit(1)
    hazards.monitor_hazards()  # Start monitoring hazards
            
if __name__ == "__main__":
    hazards = Hazards_Controller() 
    hazards.run()

        
# print(f"Distance to heat_point ({heat_point['x']}, {heat_point['y']}) is: {distance:.2f} meters")
        
