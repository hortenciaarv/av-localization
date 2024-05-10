#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointsNode(Node):
    def __init__(self):
        super().__init__('joint_node')
        self.wl = Float64()
        self.wr = Float64()
        self.wr = 0.0
        self.wl = 0.0
        self.wl_position = 0.0
        self.wr_position = 0.0
        self.v = 0
        self.w = 0
        self.l = 0.19
        self.r = 0.05
        self.dt = 0
        self.joint_msg = JointState()
        self.start_time = self.get_clock().now()
        print("The joint node is Running")
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom',self.update_odom,1)
    
    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def update_joint(self):
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_msg.name = ["right_wheel", "left_wheel"] 
        self.joint_msg.position = [self.wr_position, self.wl_position]
        self.joint_pub.publish(self.joint_msg)

    def transform(self):
        self.calculate_dt
        self.wr = (2*self.v + self.w*self.l)/(2*self.r)
        self.wl = (2*self.v - self.w*self.l)/(2*self.r)
        self.wl_position += self.wl*self.dt
        self.wr_position += self.wr*self.dt
        self.update_joint()
        self.start_time = self.get_clock().now()
    
    def update_odom(self,msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        self.transform() 



def main():
    rclpy.init()
    join_node = JointsNode()
    rclpy.spin(join_node)
    join_node.destroy_node()
    rclpy.shutdown()
