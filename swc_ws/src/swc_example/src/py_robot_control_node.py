#!/usr/bin/env python

import rospy
import math
from swc_msgs.msg import Control
from swc_msgs.srv import Waypoints

_control_pub = None

class PIDController():
    '''Oooh! Documentation! This is me copying other people's work because I don't want to import anything'''
    def __init__(self, kP, kI, kD, setpoint): #self explanatory. make sure these are fairly low
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.setpoint = setpoint
        self.accumulator = 0
        self.d_time = 0
        self.error = 0
        self.last_error = 0
    
    def reset(self): #idk when need this but here it is
        self.accumulator = 0
        self.d_time = 0
        self.error = 0
        self.last_error = 0

    def set_setpoint(self, setpoint): #prob won't need this
        self.setpoint = setpoint

    def calculate(self, curr_value): #this might work
        self.error = setpoint - curr_value
        self.d_time += 1
        self.accumulator += self.error * self.d_time
        deriv_of_error = (self.error - self.last_error) / self.d_time
        self.last_error = self.error
        return self.kP * self.error + self.accumulator *self. kI + deriv_of_error * self.kD
    # will include setters and getters for all these values and stuff later but I don't think I'll need them

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
