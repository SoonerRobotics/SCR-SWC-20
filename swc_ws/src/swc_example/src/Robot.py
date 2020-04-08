import PIDController as pid
from swc_msgs.msg import Control
import math
import tf

class Robot():
    def __init__(self, name):
        self.name = name
        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.goal_lat = 0.0
        self.goal_lon = 0.0             #                   |
        self.curr_angle = 0.0           #idk if this period V is right (like, the param)
        self.speedPID = pid.PIDController(0.5, 0.0001, 0.0001, 0.1, 0, 0.1) # we want no error/zero distance, but we have .1 tolerance
        self.anglePID = pid.PIDController(0.1, 0.000001, 0.000001, 0.1, 0, 0.1) # we want no diff between angle and angle to goal
        
    def updateCoords(self, gps):
        self.curr_lat = gps.latitude
        self.curr_lon = gps.longitude
    
    def setGoal(self, lat, lon):
        self.goal_lat = lat
        self.goal_lon = lon
    
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) # distance formula, derived from pythagorean theorum
    
    def getDist(self):
        return self.dist(self.curr_lon, self.curr_lat, self.goal_lon, self.goal_lat)
    
    def getXDist(self):
        return self.goal_lon - self.curr_lon # idk if this is right, might need to flip
    
    def getDesiredAngle(self):
        return math.asin(self.getXDist() / self.getDist()) # trig. this is _soh_cahtoa. So sin(angle) gives opposite / hypot. So asin(opposit / hypot) gives angle (asin is inverse sine)
    
    def getCurrAngle(self):
        return self.curr_angle
    
    def updateCurrAngle(self, quat):
        euler = tf.transformations.euler_from_quaternion(quat.orientation)
        heading = euler[2]
        #angle = heading.getZ()
        self.curr_angle = heading
        
    def getDesiredSpeed(self):
        return self.speedPID.calculate(self.getDist())
    
    def getAngle(self):
        return self.anglePID.calculate(self.getDesiredAngle() - self.curr_angle) #hoping this works
        
    def getAction(self):
        control_msg = Control()
        control_msg.speed = self.getDesiredSpeed()
        control_msg.turn_angle = self.getAngle()
        return control_msg
    
    # # # # # # # # # # # 3 3 3 3 # # # # # # # # #  ##  # # # # # # # # # #  #   # #  # # ###  # # # # # # # # ### # # ####### entropy