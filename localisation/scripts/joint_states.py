#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointsNode():
    def __init__(self):
        self.wl = Float64()
        self.wr = Float64()
        self.wr = 0.0
        self.wl = 0.0
        self.v = 0
        self.w = 0
        self.l = 0.19
        self.r = 0.05
        self.joint_msg = JointState()
    
        print("The joint node is Running")
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size = 10)
        self.odom_subscription = rospy.Subscriber('/wr', Odometry, self.update_odom,1)
        timer_period = 1/10  # seconds

    def update_joint(self):
        self.joint_msg.header.stamp = rospy.Time.now()
        self.joint_msg.name = ["right_wheel", "left_wheel"] 
        self.joint_msg.velocity = [self.wr, self.wl]
        self.joint_pub.publish(self.joint_msg)

    def transform(self):
        self.wr = (2*self.v + self.w*self.l)/(2*self.r)
        self.wl = (2*self.v - self.w*self.l)/(2*self.r)
        self.update_joint()
    
    def update_odom(self,msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.anguar.z
        self.transform  

def main():
    rospy.init_node('joint_node')
    JointsNode()
    rospy.spin()

if __name__ == '__main__':
    main()