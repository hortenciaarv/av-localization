import math
import time

class Bug0():
    def __init__(self):
        self.ideal_dist = 0.35
        self.real_dist = 0.0
        self.kp = 0.38 #1.8
        self.ki = 0.0
        self.error_dist = 0.0
        self.real_theta = 0.0
        self.wl_vel = 0.0
        self.wr_vel = 0.0
        self.linear_v = 0.0
        self.angular_v = 0.0
        self.r = 0.05
        self.l = 0.19
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.theta_e = 0.0
        self.integral_e_angular = 0.0

    def distance_controller(self, dt):
        self.error_dist = self.ideal_dist - self.real_dist
        print(self.error_dist)
        if (self.error_dist > 1.0):
            self.error_dist = 1.0
        if (self.error_dist < -1.0):
            self.error_dist = -1.0
        print(self.error_dist)
        self.linear_v = 0.2
        self.integral_e_angular += self.error_dist * dt
        integral = self.ki * self.integral_e_angular
        proportional = self.error_dist * self.kp

        self.angular_v = integral + proportional


    def calculate_angular_e(self):
        err_x = self.goal_x - self.pos_x
        err_y = self.goal_y - self.pos_y
        self.goal_theta = math.atan2(err_y, err_x)
        self.theta_e = self.goal_theta - self.pos_theta

