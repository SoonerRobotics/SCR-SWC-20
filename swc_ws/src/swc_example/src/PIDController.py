#!/usr/bin/env python


# Implements a PID control loop. copyed from allwpilib/wpilibj/src/main/java/edu/wpi/first/wpilibj/controller/PIDController.java
# then translated into python (I did just skip a bunch of funcs I thought I wouldn't need, we'll see)
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
        self.m_continuous = True
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
            self.m_positionError = self.getModulusError(measurement)
        else:
            self.m_positionError = self.m_setpoint - measurement
            self.m_velocityError = (self.m_positionError - self.m_prevError) / self.m_period
        
        if self.m_Ki != 0:
            # SE says max(min(my_value, max_value), min_value)
            self.m_totalError = self.clamp(self.m_totalError + self.m_positionError * self.m_period, self.m_minimumIntegral / self.m_Ki, self.m_maximumIntegral / self.m_Ki)
        
        return self.m_Kp * self.m_positionError + self.m_Ki * self.m_totalError + self.m_Kd * self.m_velocityError

   # Resets the previous error and the integral terms
    def reset(self):
        self.m_prevError = 0
        self.m_totalError = 0
    
    # end ##################################################################### no