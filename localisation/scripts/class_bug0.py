#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Bug0():
    def __init__(self):
        self.ideal_dist = 0.45
        self.real_dist = 0.0
        self.kp_wr = 2.5  #1.8
        self.kp_wl = 2.5 #1.8
        self.error_dist = 0
        self.real_theta = 0
        self.wl_vel = 0
        self.wr_vel = 0
        self.linear_v = 0
        self.angular_v = 0
        self.r = 0.05
        self.l = 0.19
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.theta_e = 0.0
    
    def calculate_velocities(self):
        self.linear_v = self.r*(self.wr_vel + self.wl_vel)/2
        self.angular_v = self.r*(self.wr_vel - self.wl_vel)/self.l

    def distance_controller(self):
        self.error_dist = self.ideal_dist - self.real_dist
        # print(self.error_dist)
        if (self.error_dist > 5):
            self.error_dist = 5
        if (self.error_dist < -5):
            self.error_dist = -5
        # print(self.error_dist)
        self.wl_vel = self.kp_wl * self.error_dist + 2.5
        self.wr_vel = -self.kp_wr * self.error_dist + 2.5
        self.calculate_velocities()
    
    def calculate_angular_e(self):
        err_x = self.goal_x - self.pos_x
        err_y = self.goal_y - self.pos_y
        self.goal_theta = math.atan2(err_y, err_x)
        self.theta_e = self.goal_theta - self.pos_theta

    def turn_right(self):
        self.wl_vel = 2
        self.wr_vel = -2
        self.calculate_velocities()

    
    


    
