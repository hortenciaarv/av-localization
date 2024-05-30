#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class KinematicNode(Node):
    def __init__(self):
        super().__init__('kinematic_node')
        self.l = 0.19
        self.r = 0.05
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_theta = 0
        self.v = 0
        self.w = 0
        self.wl = 0
        self.wr = 0
        self.wr_msg = Float32()
        self.wl_msg = Float32()
        self.pose_msg = PoseStamped()
    
        print("The kinematic node is Running")
        self.pub_pose = self.create_publisher(PoseStamped, '/pose',10)
        self.pub_wr = self.create_publisher(Float32, '/wr',1)
        self.pub_wl = self.create_publisher(Float32, '/wl',1)
        self.cmd_subscription = self.create_subscription(Twist, '/cmd_vel', self.callback_vel,10)
        
        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.update_pose)
    
    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def update_pose(self):

        self.calculate_dt()
        self.differential_drive_model()
        self.solver()
        self.transform()
        self.wr_msg.data = self.wr
        self.wl_msg.data =self.wl
        self.pose_msg.pose.position.x = self.x
        self.pose_msg.pose.position.y = self.y
        self.pose_msg.pose.position.z = self.theta

        self.pub_pose.publish(self.pose_msg)
        self.pub_wr.publish(self.wr_msg)
        self.pub_wl.publish(self.wl_msg)
        self.start_time = self.get_clock().now()

    def differential_drive_model(self):
        self.vel_x = self.v*np.cos(self.theta)
        self.vel_y = self.v*np.sin(self.theta)
        self.vel_theta = self.w

    def solver(self):
        self.x += self.vel_x*self.dt
        self.y += self.vel_y*self.dt
        self.theta += self.vel_theta*self.dt

    def transform(self):
        self.wr = (2*self.v + self.w*self.l)/(2*self.r)
        self.wl = (2*self.v - self.w*self.l)/(2*self.r)


    def callback_vel(self,msg):
        self.v = msg.linear.x
        self.w = msg.angular.z


def main():
    rclpy.init()
    kinematic_node = KinematicNode()
    rclpy.spin(kinematic_node)
    kinematic_node.destroy_node()
    rclpy.shutdown()
