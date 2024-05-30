#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection
import math


class ArucoDetection(Node):

    def __init__(self):
        super().__init__("aruco_detection_node")

        self.aruco_msg = ArucoDetection()
        self.aruco_pose = Pose()
        self.cmd_vel_msg = Twist()

        self.x_pose = 0.0
        self.z_pose = 0.0
        self.theta_pose = 0.0
        self.x_d = 0.0
        self.z_d = 0.0
        self.theta_d = 0.0

        self.kp_l = 0.6
        self.ki_l = 0.0
        self.kd_l = 0.0

        self.kp_w = 1.05
        self.ki_w = 0
        self.kd_w = 0


        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.aruco_detect)


    def calculate_lineal_e(self):
        self.prev_linear_e = self.linear_e
        self.y_e = self.z_d - self.z_pose
        self.x_e = self.x_d - self.x_pose
        self.linear_e = np.sqrt(self.x_e**2 + self.y_e**2)
        if(self.linear_e < 0.11):
            self.linear_e = 0.0

    def calculate_angular_e(self):
        self.prev_angular_e = self.angular_e
        self.theta_d = math.atan2(self.y_e,self.x_e)
        self.angular_e = self.theta_d - self.theta_pose
        if(self.angular_e < 0.11):
            self.angular_e = 0.0
    
    def calculate_errors(self):
        self.calculate_lineal_e()
        self.calculate_angular_e()
    
    def linear_controller_aruco(self):
        proportional = self.kp_l*self.linear_e 
        self.cmd_vel_msg.linear.x = proportional


    def angular_controller_arcuo(self):
        proportional = self.kp_w*self.angular_e 
        self.cmd_vel_msg.angular.z = proportional
        
    
    def activate_controller(self):
        self.calculate_errors()
        self.linear_controller_aruco()
        self.angular_controller_arcuo()

    
    def aruco_detect(self):
        for i in self.aruco_msg.markers:
            if(i.marker_id == 6): 
                self.aruco_pose = i.pose
                self.x_d = i.pose.position.x
                self.z_d = i.pose.position.z 
                self.activate_controller()
          

def main():
    rclpy.init()
    aruco_controller_node = ArucoDetection()
    rclpy.spin(aruco_controller_node)
    aruco_controller_node.destroy_node()
    rclpy.shutdown()



