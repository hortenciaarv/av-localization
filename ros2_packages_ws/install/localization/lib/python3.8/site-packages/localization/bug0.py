#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import copy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import math
import time
import sys
sys.path.append('/home/puzzlebot/ros2_packages_ws/src/localization/localization/')
from class_controller import Controller
from class_bug0 import Bug0


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
     
        return yaw_z # in radians

class ToGoalNode(Node):
    def __init__(self):
        super().__init__("toGoalNode")

        #variables
        self.x_d = 0
        self.y_d = 0
        self.theta_d = 0
        self.x_pose = 0
        self.y_pose = 0
        self.theta_pose = 0
        self.x_pose_prev = 0
        self.y_pose_prev = 0
        self.theta_pose_prev = 0
        self.wl = 0.0
        self.wr = 0.0
        self.prev_distance = 10000
        self.ranges = []
        self.flag = False
        self.flag_set_point = False
        self.controller_flag = True

        #messages
        self.cmd_vel_msg = Twist()
        self.controller = Controller()
        self.bug0 = Bug0()

        print("Running To Goal Node")

        #susbscribers
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb ,10)   
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10) 
        self.sub_set_point = self.create_subscription(Pose, "/set_point", self.setPoint_cb, 10)   

        #publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel",1)

        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.run_bug0)


    # callbacks
    def odom_cb(self, msg):

        self.x_pose_prev = self.x_pose
        self.y_pose_prev = self.y_pose
        self.theta_pose_prev = self.theta_pose

        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        self.theta_pose = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

        self.bug0.pos_x = self.x_pose
        self.bug0.pos_y = self.y_pose
        self.bug0.pos_theta = self.theta_pose
        
        self.controller.x_pose = self.x_pose
        self.controller.y_pose = self.y_pose
        self.controller.theta_pose = self.theta_pose


    def scan_cb(self, msg):
        self.ranges = msg.ranges

    def setPoint_cb(self, msg):
        self.x_d = msg.position.x
        self.y_d = msg.position.y

        self.bug0.goal_x = self.x_d
        self.bug0.goal_y = self.y_d
        
        self.controller.x_d = self.x_d
        self.controller.y_d = self.y_d

        self.flag_set_point = True

    #dt calculate
    def calculate_dt(self):
        self.current_time = self.start_time
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def update_velocities_bug0(self):
        self.cmd_vel_msg.linear.x = self.bug0.linear_v
        self.cmd_vel_msg.angular.z = self.bug0.angular_v

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    #stop
    def stop(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        print("stoping")

    #forward
    def forward(self):
        self.cmd_vel_msg.linear.x = 0.8
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    #turn right
    def turn_left(self):
        print("girando")
        # self.bug0.wl_vel = 0.3
        # self.bug0.wr_vel = -0.3
        # self.bug0.calculate_velocities()
        # self.update_velocities_bug0()
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.5
        self.cmd_vel_pub.publish(self.cmd_vel_msg)


    def bug0_algorithm(self):
        pass
        
    def follow_wall(self):
        print("wall")
        self.bug0.calculate_angular_e()

        while(self.bug0.theta_e != 0.0 or self.bug0.theta_e != -0.0):
            self.bug0.real_dist = self.ranges[810]
            print("Enfrente", self.ranges[0])
            print("A lado", self.ranges[810])
            for i in range(0,2):
                if (self.ranges[i] <= 0.45):
                    self.stop()
                    time.sleep(1)
                    self.flag = True
                    break
            if not self.flag: 
                for i in range(1078,1080):
                    if (self.ranges[i] <= 0.45):
                        self.stop()
                        time.sleep(1)
                        self.flag = True
                        break
            if self.flag:
                self.flag = False
                return
            # print("Error angular: ", self.bug0.theta_e)
            self.bug0.distance_controller(self.dt)
            print("Error dist: ", self.bug0.error_dist)
            self.update_velocities_bug0()
            self.bug0.calculate_angular_e()
        


    def controller_algorithm(self):
        print("controller")
        self.controller.update_vel(self.dt)
        self.cmd_vel_msg = self.controller.cmd_vel_msg
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def run_bug0(self):
        # if self.flag_set_point:
        self.calculate_dt()
        if(len(self.ranges)>0):
            for i in range(0,2):
                if (self.ranges[i] <= 0.45):
                    print("Detecte algo a la izquierda")
                    self.stop()
                    self.flag = True
                    break
            if not self.flag: 
                for i in range(1078,1080):
                    if (self.ranges[i] <= 0.45):
                        print("detecte algo a la derecha")
                        self.stop()
                        self.flag = True
                        break

            if self.flag:
                time.sleep(1)
                self.turn_left()
                time.sleep(0.5)
                self.stop()
                self.flag = False
                self.ranges[0] = 0.7
                self.ranges[1] = 0.7
                self.ranges[1078] = 0.7
                self.ranges[1079] = 0.7
                time.sleep(1)
                self.follow_wall()
            else:
                self.controller_algorithm()
                    
        # self.flag_set_point = False

def main():
    rclpy.init()
    to_goal_node = ToGoalNode()
    rclpy.spin(to_goal_node)
    to_goal_node.destroy_node()
    rclpy.shutdown()

