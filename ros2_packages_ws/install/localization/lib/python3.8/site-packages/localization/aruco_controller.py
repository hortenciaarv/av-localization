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
import time



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

class ArucoController(Node):

    def __init__(self):
        super().__init__("aruco_detection_node")

        self.aruco_pose = Pose()
        self.cmd_vel_msg = Twist()

        self.x_pose = 0.0
        self.z_pose = 0.0
        self.theta_pose = 0.0
        self.x_d = 0.0
        self.z_d = 0.0
        self.theta_d = 0.0

        self.linear_e = 0.0
        self.prev_linear_e = 0.0
        self.angular_e = 0.0
        self.prev_angular_e = 0.0
        self.integral_e_linear = 0.0
        self.integral_e_angular = 0.0
        self.x_e = 0.0
        self.y_e = 0.0


        self.kp_l = 0.7
        self.ki_l = 0.0
        self.kd_l = 0.0

        self.kp_w = 1.05
        self.ki_w = 0
        self.kd_w = 0

        self.flag_setpoint = True

        print("Aruco controller is running")

        # self.odom_sub = self.create_subscription(Odometry, '/odom',self.pose_update,1)
        self.aruco_subscription = self.create_subscription(ArucoDetection,'/aruco_detections',self.aruco_detect,1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)


        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.update_aruco_controller)


    def calculate_lineal_e(self):
        self.prev_linear_e = self.linear_e
        self.y_e = self.z_d - self.z_pose
        self.x_e = self.x_d - self.x_pose
        self.linear_e = np.sqrt(self.x_e**2 + self.y_e**2)
        print("error lin: ", self.linear_e)

        if(self.linear_e < 0.08):
            self.linear_e = 0.0

    def calculate_angular_e(self):
        self.prev_angular_e = self.angular_e
        self.theta_d = math.atan2(self.y_e,self.x_e)
        self.angular_e = self.theta_d - self.theta_pose
        print("error ang: ", self.angular_e)

        if(abs(self.angular_e) < 0.05):
            self.angular_e = 0.0
    
    def calculate_errors(self):
        self.calculate_lineal_e()
        # self.calculate_angular_e()
    
    def linear_controller_aruco(self):
        proportional = self.kp_l*self.linear_e 
        self.cmd_vel_msg.linear.x = proportional

        if(self.cmd_vel_msg.linear.x > 0.3):
            self.cmd_vel_msg.linear.x = 0.3
        
        if(self.cmd_vel_msg.linear.x < -0.3):
            self.cmd_vel_msg.linear.x = -0.3


    def angular_controller_arcuo(self):
        proportional = self.kp_w*self.angular_e 
        self.cmd_vel_msg.angular.z = proportional

        if(self.cmd_vel_msg.angular.z > 0.3):
            self.cmd_vel_msg.angular.z = 0.3
        
        if(self.cmd_vel_msg.angular.z < -0.3):
            self.cmd_vel_msg.angular.z = -0.3
        
    
    def activate_controller(self):
        self.calculate_errors()
        self.linear_controller_aruco()
        # self.angular_controller_arcuo()
        self.flag_setpoint = True

    # def pose_update(self, msg):
    #     self.x_pose = msg.pose.pose.position.x
    #     self.z_pose = msg.pose.pose.position.y
    #     self.theta_pose = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
    #                                             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
    
    def aruco_detect(self, msg):
        
        for i in msg.markers:
            if(i.marker_id == 6): 
                self.aruco_pose = i.pose
                self.x_d = i.pose.position.z
                self.z_d = i.pose.position.x
                print("x: ", self.x_d)
                print("y: ", self.z_d)
                break
            else:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.cmd_vel_msg)
                self.linear_e = 0
                
                

    def update_aruco_controller(self):
        self.activate_controller()
        self.pub_cmd_vel.publish(self.cmd_vel_msg)
        self.start_time = self.get_clock().now()

def main():
    rclpy.init()
    aruco_controller_node = ArucoController()
    rclpy.spin(aruco_controller_node)
    aruco_controller_node.destroy_node()
    rclpy.shutdown()



