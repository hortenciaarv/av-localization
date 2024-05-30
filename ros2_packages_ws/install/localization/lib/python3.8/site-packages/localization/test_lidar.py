#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class TestLidar(Node):
    def __init__(self):
        super().__init__('test_lidar_node')

        self.ranges = []
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)


    def scan_cb(self, msg):
        self.ranges = msg.ranges
        print(len(self.ranges))
        print("0: ", self.ranges[0])
        print("270: ", self.ranges[270])
        print("540: ", self.ranges[540])
        print("810: ", self.ranges[810])


def main():
    rclpy.init()
    test_lidar_node = TestLidar()
    rclpy.spin(test_lidar_node)
    test_lidar_node.destroy_node()
    rclpy.shutdown()







