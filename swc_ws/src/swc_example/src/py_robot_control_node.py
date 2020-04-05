#!/usr/bin/env python

import rospy
import math
from swc_msgs.msg import Control
from swc_msgs.srv import Waypoints

_control_pub = None

# Implements a PID control loop. copyed from allwpilib/wpilibj/src/main/java/edu/wpi/first/wpilibj/controller/PIDController.java
# then translated into python (I did just skip a bunch of funcs I thought I wouldn't need, we'll see
class PIDController():
    def __init__(self, Kp, Ki, Kd, period, setpoint, posTolerance):
        self.m_Kp = Kp
        self.m_Ki = Ki
        self.m_Kd = Kd
        self.m_period = period
        self.m_maximumIntegral = 1.0
        self.m_minimumIntegral = -1.0
        self.m_maximumInput = 1.0
        self.m_minimumInput = -1.0
        self.m_continuous = False
        self.m_positionError = 0.0
        self.m_velocityError = 0.0
        self.m_prevError = 0.0
        self.m_totalError = 0.0
        self.m_positionTolerance = posTolerance
        self.m_velocityTolerance = float("inf")
        self.m_setpoint = setpoint

    def setPID(self, Kp, Ki, Kd):
        self.m_Kp = Kp
        self.m_Ki = Ki
        self.m_Kd = Kd

    def setSetpoint(self, setpoint):
        self.m_setpoint = setpoint
  
    def getSetpoint(self):
        return self.m_setpoint
    
    def atSetpoint(self):
        return math.abs(self.m_positionError) < m_positionTolerance and math.abs(self.m_velocityError) < m_velocityTolerance
 
    def enableContinuousInput(self, minimumInput, maximumInput):
        self.m_continuous = True;
        self.m_minimumInput = minimumInput
        self.m_maximumInput = maximumInput
 
    def disableContinuousInput(self):
        self.m_continuous = False

    def getModulusError(self, measurement):
        modulus = self.maximumInput - self.minimumInput
        error = self.setpoint % modulus - measurement % modulus
        return (error - self.minimumInput) % modulus + self.minimumInput
        
    def clamp(self, value, max_, min_):
        return max(min(value, max_), min_)
    
    def calculate(self, measurement):
        self.m_prevError = self.m_positionError

        if self.m_continuous:
            self.m_positionError = getModulusError(measurement)
        else:
            self.m_positionError = self.m_setpoint - measurement
            self.m_velocityError = (self.m_positionError - self.m_prevError) / self.m_period
        
        if m_Ki != 0:
            # SE says max(min(my_value, max_value), min_value)
            self.m_totalError = clamp(self.m_totalError + self.m_positionError * self.m_period, self.m_minimumIntegral / self.m_Ki, self.m_maximumIntegral / self.m_Ki)
        
        return self.m_Kp * self.m_positionError + self.m_Ki * self.m_totalError + self.m_Kd * self.m_velocityError;

   # Resets the previous error and the integral terms
  def reset(self)
    self.m_prevError = 0;
    self.m_totalError = 0;
    
    # end ##################################################################### no
    
class Robot():
    def __init__(self, name):
        self.name = name
        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.goal_lat = 0.0
        self.goal_lon = 0.0             #                   |
        self.curr_angle = 0.0           #idk if this period V is right (like, the param)
        self.speedPID = PIDController(0.5, 0.0001, 0.0001, 0.1, 0, 0.1) # we want no error/zero distance, but we have .1 tolerance
        self.anglePID = PIDController(0.1, 0.000001, 0.000001, 0.1, 0, 0.1) # we want no diff between angle and angle to goal
        
    def updateCoords(self, lat, lon):
        self.curr_lat = lat
        self.curr_lon = lon
    
    def setGoal(self, lat, lon):
        self.goal_lat = lat
        self.goal_lon = lon
    
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
    def getDist(self):
        return self.dist(self.curr_lon, self.curr_lat, self.goal_lon, self.goal_lat)
    
    def getXDist(self):
        return self.goal_lon - self.curr_lon #idk if this is right, might need to flip
    
    def getDesiredAngle(self):
        return math.asin(self.getXDist(), self.getDist())
    
    def getCurrAngle(self):
        return self.curr_angle
    
    def updateCurrAngle(self, angle):
        self.curr_angle = angle
        
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
def timer_callback(event):
    # Create a new message with speed 1 (m/s) and turn angle 15 (degrees CW)
    control_msg = Control()
    control_msg.speed = 2 #so we go super speed
    control_msg.turn_angle = 15

    # Publish the message to /sim/control so the simulator receives it
    _control_pub.publish(control_msg)


def main():
    global _control_pub

    # Initalize our node in ROS
    rospy.init_node('py_robot_control_node')

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()

    print(waypoints.waypoints)

    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
