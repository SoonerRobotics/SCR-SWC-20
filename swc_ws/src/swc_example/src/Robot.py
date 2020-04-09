import PIDController as pid
from swc_msgs.msg import Control
import math
import tf

class Robot():
    def __init__(self, name, waypoints):
        self.name = name
        self.curr_lat = waypoints[0].latitude # have us start out at first location
        self.curr_lon = waypoints[0].longitude
        self.target_waypoints = waypoints
        self.curr_angle = 0.0
        #self.speedPID = pid.PIDController(10, 0.0001, 0.0001, 0.1, 0, 0.1) # we want no error/zero distance, but we have .1 tolerance
        #self.anglePID = pid.PIDController(100, 0, 0, 0.1, 0, 0.1) # we want no diff between angle and angle to goal
        self.updateTarget(1) # to set first goal
        self.target_index = 1 # start with bonus waypoint as 1st target

    # updates the robot's current position and handle goal waypoint stuff
    def updateCoords(self, gps):
        self.curr_lat = gps.latitude
        self.curr_lon = gps.longitude
        if self.curr_lat > self.target_waypoints[1].latitude and self.target_index == 1: # if we've made it past 1st waypoint
            self.target_index += 1 # go to next waypoint
            self.updateTarget(self.target_index)
        elif self.curr_lat > self.target_waypoints[2].latitude and self.target_index == 2:
            self.target_index += 1 # go for next
            self.updateTarget(self.target_index)
        elif self.curr_lat > self.target_waypoints[3].latitude and self.target_index == 3:
            self.target_index += 1
            self.updateTarget(self.target_index)

    def updateTarget(self, index):
        self.goal_lat = self.target_waypoints[index].latitude
        self.goal_lon = self.target_waypoints[index].longitude
    
    # implimentation of distance formula, for internal use
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) # distance formula, derived from pythagorean theorum
    
    # returns distance between robot and goal
    def getDist(self):
        return self.dist(self.curr_lon, self.curr_lat, self.goal_lon, self.goal_lat)
    
    # needed for the angle that we want to drive at
    def getXDist(self):
        return self.goal_lon - self.curr_lon # idk if this is right, might need to flip
    
    # gives us the angle that we want to get to
    def getDesiredAngle(self):
        return math.asin(self.getXDist() / self.getDist()) # trig. this is _soh_cahtoa. So sin(angle) gives opposite / hypot. So asin(opposit / hypot) gives angle (asin is inverse sine)
    
    # updates our current angle from IMU data
    def updateCurrAngle(self, data):
        quat = data.orientation # we're interested in orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
        euler = tf.transformations.euler_from_quaternion(explicit_quat) # get a euler
        self.curr_angle = euler[2] # and update the yaw
    
    # final function call for speed
    def getDesiredSpeed(self):
        return self.getDist() * 200000 # it's WORKING! but honestly you don't need a PID for this. Just go fast lol
        
    # final function call for angle
    def getAngle(self):
        return (self.curr_angle - self.getDesiredAngle()) * 5 # It's WORKING! but anglePID is messed up. And we understeer a lot
        # that's why we have a x3
        
    # function that actually gives the control() message
    def getAction(self):
        control_msg = Control()
        control_msg.speed = self.getDesiredSpeed()
        control_msg.turn_angle = self.getAngle()
        return control_msg
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #