#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from rclpy.node import Node
from nav_msgs.msg import Odometry

class localizationPublisher(Node):
    def __init__(self):
        super().__init__('localization')
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
        self.odom_msg = Odometry()
    
        print("The localization node is Running")
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)
        self.wr_subscription = self.create_subscription(Float32, '/wr',self.wr_update,1)
        self.wl_subscription = self.create_subscription(Float32, '/wl',self.wl_update,1)
        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.update_odom)

    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def update_odom(self):
        

        self.calculate_dt()
        self.transform()
        self.differential_drive_model()
        self.solver()

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.w = self.theta
        self.odom_msg.twist.twist.linear.x = self.v
        self.odom_msg.twist.twist.angular.z = self.vel_theta

        self.publisher_odom.publish(self.odom_msg)
        self.start_time = self.get_clock().now()

    def differential_drive_model(self):
        self.vel_x = self.v*np.cos(self.theta)
        self.vel_y = self.v*np.sin(self.theta)
        self.vel_theta =self.w

    def solver(self):
        self.x += self.vel_x*self.dt
        self.y += self.vel_y*self.dt
        self.theta += self.vel_theta*self.dt

    def transform(self):
        self.v = self.r*(self.wr + self.wl)/2
        self.w = self.r*(self.wr - self.wl)/self.l
    
    def wr_update(self,msg):
        self.wr = msg.data    

    def wl_update(self,msg):
        self.wl = msg.data


def main():
    rclpy.init()
    localization_node = localizationPublisher()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()
