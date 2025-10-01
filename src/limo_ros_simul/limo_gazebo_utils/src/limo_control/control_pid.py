        
#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

# current_dir = os.path.dirname(os.path.abspath(__file__))
# parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
# if parent_dir not in sys.path:
#     sys.path.insert(0, parent_dir)
# from time import *

class PIDController:
    def __init__(self, Kp=0.60, Ki=0.0, Kd=0.0527): # Kp = 0.64 Kd = 0.0527
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time()
        self.integral_limit = 100.0

    def compute(self, error):
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0:
            dt = 1e-6

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output
