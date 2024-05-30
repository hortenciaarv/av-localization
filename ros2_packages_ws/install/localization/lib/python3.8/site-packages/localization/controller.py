#!/usr/bin/env python3
import rclpy
import numpy as np
from geometry_msgs.msg import Pose
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # roll_x = math.atan2(t0, t1)
     
        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

class ControlNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.x_d = 0
        self.y_d = 0
        self.theta_d = 0
        self.x_pose = 0
        self.y_pose = 0
        self.theta_pose = 0

        self.linear_e = 0
        self.prev_linear_e = 0
        self.angular_e = 0
        self.prev_angular_e = 0
        self.integral_e_linear = 0
        self.integral_e_angular = 0
        self.x_e = 0
        self.y_e = 0

        self.kp_l = 0.6
        self.ki_l = 0.0
        self.kd_l = 0.0

        self.kp_w = 1.05
        self.ki_w = 0
        self.kd_w = 0

        self.cmd_vel_msg = Twist()
    
        print("The control node is Running")
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom',self.pose_update,1)
        self.set_point_sub = self.create_subscription(Pose, '/set_point',self.set_point_update,1)
        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.update_vel)

    def calculate_lineal_e(self):
        self.prev_linear_e = self.linear_e
        self.y_e = self.y_d - self.y_pose
        self.x_e = self.x_d - self.x_pose
        self.linear_e = np.sqrt(self.x_e**2 + self.y_e**2)
        if(self.linear_e < 0.12):
            self.linear_e = 0.0

    def calculate_angular_e(self):
        self.prev_angular_e = self.angular_e
        self.theta_d = math.atan2(self.y_e,self.x_e)
        self.angular_e = self.theta_d - self.theta_pose
        if(self.angular_e < 0.12):
            self.angular_e = 0.0
    
    def calculate_errors(self):
        self.calculate_lineal_e()
        self.calculate_angular_e()

    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def lineal_controller(self):
        proportional = self.kp_l*self.linear_e 

        self.integral_e_linear += self.linear_e*self.dt
        integral = self.ki_l*self.integral_e_linear

        derivate = ((self.linear_e - self.prev_linear_e)/self.dt)*self.kd_l

        self.cmd_vel_msg.linear.x = proportional + integral + derivate 
        if(self.cmd_vel_msg.linear.x > 2):
            self.cmd_vel_msg.linear.x = 2.0
        # elif(self.cmd_vel_msg.linear.x < 0.47 and self.cmd_vel_msg.linear.x > 0):
        #     self.cmd_vel_msg.linear.x = 0.0

        

    
    def angular_controler(self):
        proportional = self.kp_w*self.angular_e 

        self.integral_e_angular += self.angular_e*self.dt
        integral = self.ki_w*self.integral_e_angular

        derivate = ((self.angular_e - self.prev_angular_e)/self.dt)*self.kd_w
        
        self.cmd_vel_msg.angular.z = proportional + integral + derivate
        if(self.cmd_vel_msg.angular.z > 2):
            self.cmd_vel_msg.angular.z = 2.0
        # if(self.cmd_vel_msg.angular.z < 0.47 and self.cmd_vel_msg.linear.x > 0):
        #     self.cmd_vel_msg.angular.z = 0.0 
    
    def controller(self):
        self.calculate_errors()
        self.lineal_controller()
        self.angular_controler()

    def update_vel(self):
        self.calculate_dt()
        self.controller()
        self.pub_cmd_vel.publish(self.cmd_vel_msg)
        self.start_time = self.get_clock().now()

    def pose_update(self, msg):
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        self.theta_pose = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)


    def set_point_update(self, msg):
        self.x_d = msg.position.x
        self.y_d = msg.position.y

    

def main():
    rclpy.init()
    controller_node = ControlNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()