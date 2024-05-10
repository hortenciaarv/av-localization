#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.node import Node
import time

# impulse = 35.0
#Strength of the impulse
impulse =  35.0

#Duration between impulses
duration = 20.0



def stop():
    print("stopping sim")


def main():
    rclpy.init()
    node = rclpy.create_node("impulse_generator")
    rate = node.create_rate(100)
    pub = node.create_publisher(Float32, '/tau',10)
    msg = Float32()

    #Get Parameters   

    print("The impulse generator is Running")
    while rclpy.ok():

        msg.data = impulse
        pub.publish(msg)

        time.sleep(0.1)

        msg.data = 0.0
        pub.publish(msg)

        time.sleep(duration)

        #Wait and repeat
        rclpy.spin_once(node)