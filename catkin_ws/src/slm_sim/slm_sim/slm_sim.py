#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.node import Node

k = 0.01
m = 0.75
l = 0.36
g = 9.8
Tau = 0.0
a = l/2
J = (4/3)*m*a**2


def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi), (2*np.pi))
    if(result < 0):
        result += 2 * np.pi
    
    return result - np.pi

def update_position(x1, x2):
    x1 += x2 * 1/100
    x2_dot = (1/J)*(Tau - m*g*a*np.cos(x1) - k*x2)
    x2 += x2_dot*1/100

    return x1, x2

def main():
    rclpy.init()
    node = rclpy.create_node("SLM_sim")
    rate = node.create_rate(100)
    pub = node.create_publisher(JointState, '/joint_states',10)
    msg = JointState()

    x1 = 0.0
    x2 = 0.0
    print("The SLM sim is Running")

    try:
        while rclpy.ok():
            x1, x2 = update_position(x1, x2)
            tetha = wrap_to_Pi(x1)

            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = ["joint2"] 
            msg.position = [tetha]
            msg.velocity = [x2]

            pub.publish(msg)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()