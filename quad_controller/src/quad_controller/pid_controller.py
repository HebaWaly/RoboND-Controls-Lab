# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        
        # Set max wind up
        self.max_windup_ = float(max_windup)

        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        #self.start_time_ = start_time
        self.error_sum_ = 0.0
        self.last_error_ = 0.0

        
    def reset(self):
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)

    def setKI(self, ki):
        self.ki_ = float(ki)

    def setKD(self, kd):
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        self.max_windup_ = float(max_windup)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            # Delta time is zero
            return 0
        
        # Calculate the error 
        error = self.set_point_ - measured_value
        
        # Set the last_timestamp_ 
        self.last_timestamp_ = timestamp

        # Sum the errors
        self.error_sum_ += error * delta_time
        
        # Update the past error
        self.last_error_ = error
        
        # Find delta_error
        delta_error = error - self.last_error_
        
        # Update the past error
        self.last_error_ = error
        
        # Address max windup
        ########################################
        #if(self.error_sum_ > self.max_windup_):
         #   self.error_sum_ = self.max_windup_
        #elif(self.error_sum_ < -self.max_windup_):
         #   self.error_sum_ = -self.max_windup_
        ########################################
        
        # Proportional error
        p = self.kp_ * error
       
        # Integral error
        i = self.ki_ * self.error_sum_
       
        # Recalculate the derivative error here incorporating 
        # derivative smoothing!
        ########################################
        #d_error = self.alpha * delta_error/delta_time + (1 - self.alpha) * self.last_d_error
        d = self.kd_ * delta_error/delta_time
        #self.last_d_error = d_error
        ########################################
        
        # Set the control effort
        u = p + i + d
        
        # Enforce actuator saturation limits
        ########################################
        #if(u > self.umax):
         #   u = self.umax
        #elif(u < self.umin):
         #   u = self.umin
        ########################################
        
        return u


