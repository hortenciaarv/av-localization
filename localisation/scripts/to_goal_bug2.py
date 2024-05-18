#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import math
import sys
sys.path.append('/root/workspace/puzzlebot_ws/src/localisation/scripts/')
from class_controllers import Controller
from class_bug2 import Bug2


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

class ToGoalNode():
    def __init__(self):
        rospy.init_node("toGoalNode")

         #rospy.init_node('controller_node')
        self.x_d = 0
        self.y_d = 0
        self.theta_d = 0
        self.x_pose = 0
        self.y_pose = 0
        self.theta_pose = 0
        self.wl = 0.0
        self.wr = 0.0
        self.prev_distance = 10000
        self.ranges = []
        self.cmd_vel_msg = Twist()
        self.controller = Controller()
        self.bug2 = Bug2()
        self.hit_point = (0,0)
        self.leave_point = (0,0)


        print("Running To Goal Node")

        rospy.Subscriber("/puzzlebot_1/base_controller/odom", Odometry, self.odom_cb ,queue_size = 10)   
        rospy.Subscriber("/puzzlebot_1/scan", LaserScan, self.scan_cb, queue_size = 10) 
        rospy.Subscriber("/set_point", Pose, self.setPoint_cb, queue_size = 10)   

        self.cmd_vel_pub = rospy.Publisher("/puzzlebot_1/base_controller/cmd_vel", Twist,queue_size = 10)

        self.start_time = rospy.Time.now()
        timer_period = 1/10  # seconds
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.run)


    def odom_cb(self, msg):
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        self.theta_pose = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

        self.bug2.pos_x = self.x_pose
        self.bug2.pos_y = self.y_pose
        self.bug2.pos_theta = self.theta_pose
        
        self.controller.x_pose = self.x_pose
        self.controller.y_pose = self.y_pose
        self.controller.theta_pose = self.theta_pose

    def calculate_dt(self):
        self.current_time = self.start_time
        self.duration = rospy.Time.now() - self.start_time
        self.dt = self.duration

    def scan_cb(self, msg):
        self.ranges = msg.ranges

    def setPoint_cb(self, msg):
        self.x_d = msg.position.x
        self.y_d = msg.position.y

        self.bug2.goal_x = self.x_d
        self.bug2.goal_y = self.y_d
        
        self.controller.x_d = self.x_d
        self.controller.y_d = self.y_d

        self.bug2.calculate_line_param()

    def stop(self):
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def update_velocities_bug2(self):
        self.cmd_vel_msg.linear.x = self.bug2.linear_v
        self.cmd_vel_msg.angular.z = self.bug2.angular_v
        # print(self.cmd_vel_mssg)
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def bug2_algorithm(self):
        print("bug2")
        while(self.ranges[270] >= 0.31 or self.ranges[180] <= 0.6):
            # print("bug2")
            print("Izq in while", self.ranges[270])
            print("Del in while",self.ranges[180])
            self.bug2.turn_right()
            self.update_velocities_bug2()
        self.stop()
        self.hit_point = (self.x_pose, self.y_pose)
        print("Izq after while", self.ranges[270])
        print("Del after while", self.ranges[180])


    
    def follow_wall(self):
        print("wall")
        flag = False
        y_line =self.bug2.calculate_line()

        self.distance_hit_point = (self.y_d - self.hit_point[1])/(self.x_d - self.hit_point[0])
        self.distance_pose = (self.y_d - self.y_pose)/(self.x_d - self.x_pose)
        
        while(abs(self.y_pose - y_line) > 0.12 or self.distance_hit_point < self.distance_pose):
            for i in range(150,211):
                if (self.ranges[i] <= 0.35):
                    self.stop()
                    flag = True
                    break
            if flag:
                break
            # print(self.bug2.theta_e)
            self.bug2.real_dist = self.ranges[270]
            self.bug2.distance_controller()
            self.update_velocities_bug2()
            # selff.bug2.calculate_line()
        

    def controller_algorithm(self):
        print("controller")
        self.controller.update_vel(self.dt)
        self.cmd_vel_msg = self.controller.cmd_vel_msg
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    
    def run(self, event):
        flag = False
        self.calculate_dt()
        print("Run corriendo")
        if(len(self.ranges)>0):
            for i in range(150,211):
                if (self.ranges[i] <= 0.45):
                    flag = True
                    break
                
            if flag:
                self.stop()
                self.bug2_algorithm()
                self.follow_wall()
            else:
                self.controller_algorithm()

        self.start_time = rospy.Time.now()

def main():

    ToGoalNode()
    rospy.spin()

if __name__ == '__main__':
    main()





    


