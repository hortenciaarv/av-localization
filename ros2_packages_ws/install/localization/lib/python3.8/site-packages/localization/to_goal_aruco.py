#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import copy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from aruco_opencv_msgs.msg import ArucoDetection


import math
import time
import sys
sys.path.append('/home/puzzlebot/Puzzlebot_Lidar_ROS1_ROS2/ros2_packages_ws/src/localization/localization')
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

        #set point
        self.x_d = 2.0
        self.y_d = 2.0
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
        self.flag_set_point = False
        self.cmd_vel_msg = Twist()
        self.controller = Controller()
        self.controller.x_d = self.x_d
        self.controller.y_d = self.y_d
        self.bug0 = Bug0()
        self.bug0.goal_x = self.x_d
        self.bug0.goal_y = self.y_d
        self.controller_flag = True
        self.state = 'controller'
        self.lidar_flag = False
        self.station_input = 'marker_1'
        self.aruco_cube = False
        self.aruco_station = False
        self.prev_state = self.state

        # aruco controller
        self.controller_aruco = Controller()
        self.controller_aruco.angular_error_min = 0.35
        self.controller_aruco.linear_error_min = 0.063
        self.controller_aruco.kp_l = 0.8
        self.controller_aruco.kp_w = 1.3

        self.aruco_detected = False
        

        print("Running To Goal Node")

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'marker_6').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Create publisher and suscribers
        self.create_subscription(Odometry, "/odom", self.odom_cb ,10)   
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10) 
        self.create_subscription(Pose, "/set_point", self.setPoint_cb, 10) 
        self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_cb, 10)  

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel",1)
        self.gripper_pub = self.create_publisher(Float32, "/ServoAngle",10)

        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.run)

    
    def aruco_cb(self,msg):
        for i in msg.markers:
            if(i.marker_id == 6 and self.state == "find cube"):
                self.aruco_cube = True
                # self.stop()
                # time.sleep(3)
            elif(i.marker_id == 1 and self.state == "find station"):
                self.aruco_station = True

            

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

        self.controller_aruco.x_pose = self.x_pose
        self.controller_aruco.y_pose = self.y_pose
        self.controller_aruco.theta_pose = self.theta_pose

    def calculate_dt(self):
        self.current_time = self.start_time
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def scan_cb(self, msg):
        self.ranges = msg.ranges
        self.lidar_flag = True

    def setPoint_cb(self, msg):
        self.x_d = msg.position.x
        self.y_d = msg.position.y

        self.bug0.goal_x = self.x_d
        self.bug0.goal_y = self.y_d
        
        self.controller.x_d = self.x_d
        self.controller.y_d = self.y_d

        self.flag_set_point = True

    def stop(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        print("stoping")

    def update_velocities_bug0(self):
        self.cmd_vel_msg.linear.x = self.bug0.linear_v
        self.cmd_vel_msg.angular.z = self.bug0.angular_v

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def check_wall(self):
        for i in range(0,2):
            if (self.ranges[i] <= 0.45):
                self.stop()
                time.sleep(1)
                return True
        for i in range(1078,1080):
            if (self.ranges[i] <= 0.45):
                self.stop()
                time.sleep(1)
                return True
        return False
    
    def turn(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.5 #0.5
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def follow_wall(self):
        self.bug0.calculate_angular_e()
        print(self.bug0.theta_e)
        self.bug0.real_dist = self.ranges[810]
        # print("Enfrente en wall", self.ranges[0])
        # print("A lado en wall", self.ranges[810])
        if (abs(self.bug0.theta_e) < 0.25):
            self.state = 'controller'
        else:
            self.bug0.distance_controller(self.dt)
            self.update_velocities_bug0()

    def controller_alg(self):
        self.controller.update_vel(self.dt)
        self.cmd_vel_msg = self.controller.cmd_vel_msg
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def find_aruco(self):

        from_frame_rel = self.target_frame
        to_frame_rel = 'odom'

        msg = Twist()

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.aruco_detected = True
            
            self.controller_aruco.x_d = t.transform.translation.x
            self.controller_aruco.y_d = t.transform.translation.y

            self.controller_aruco.controller()
            print("Error lineal: ", self.controller_aruco.linear_e)
            print("Error angular: ", self.controller_aruco.angular_e)

            msg.linear.x = self.controller_aruco.cmd_vel_msg.linear.x
            msg.angular.z = self.controller_aruco.cmd_vel_msg.angular.z

            self.cmd_vel_pub.publish(msg)

            # if self.controller_aruco.linear_e == 0 and self.state=="find cube":
            #     self.state = "close gripper"
            #     self.stop()
            #     time.sleep(2)
            #     self.aruco_detected = False
            #     self.aruco_cube = False
            
            if self.controller_aruco.linear_e == 0 and self.state=="find station":
                self.state = "open gripper"
                self.stop()
                time.sleep(2)
                self.aruco_detected = False
                self.aruco_station = False

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.aruco_detected and self.state=="find cube":
                self.state = "close gripper"
                self.stop()
                time.sleep(1)
                self.aruco_detected = False
                self.aruco_cube = False

            if self.aruco_detected and self.state=="find station":
                self.state = "open gripper"
                self.stop()
                time.sleep(2)
                self.aruco_detected = False
                self.aruco_station = False

            self.cmd_vel_pub.publish(msg)

    def close_gripper(self):
        msg = Float32()
        msg.data = -20.0
        self.gripper_pub.publish(msg)
        time.sleep(2)
        self.state = "controller"

    def open_gripper(self):
        msg = Float32()
        msg.data = 180.0
        self.gripper_pub.publish(msg)
        time.sleep(2)
        self.state = "stop"

    def run(self):
        # aruco
        # Store frame names in variables that will be used to
        # compute transformations
        if self.state == "find cube":
            self.target_frame = 'marker_6'
            if self.aruco_cube:
                self.find_aruco()
            else:
                self.turn()
                time.sleep(0.2)
                self.stop()
                time.sleep(1)
            # self.find_aruco()
        
        if self.state == "find station":
            self.controller_aruco.kp_l = 0.9
            self.controller_aruco.kp_w = 1.6
            self.controller_aruco.linear_error_min = 0.08
            if self.aruco_station:
                self.target_frame = self.station_input
                self.find_aruco()
            else:
                self.turn()
                time.sleep(0.2)
                self.stop()
                time.sleep(1)
        if self.state == "close gripper":
            self.close_gripper()
        
        if self.state == "open gripper":
            self.open_gripper()
            
        # bug0
        if self.lidar_flag:
            # print("Enfrente en run ", self.ranges[0])
            # print("A lado en run ", self.ranges[810])
            self.calculate_dt()
            if self.check_wall() and (self.state == 'follow_wall' or self.state == 'controller') :
                self.state = 'turn'
            print(self.state)

            if (self.state == 'turn'):
                self.turn()
                time.sleep(0.5)
                self.stop()
                time.sleep(1)
                self.state = 'follow_wall'
            elif(self.state == 'follow_wall'):
                self.follow_wall()
            elif(self.state == 'controller'):
                self.controller_alg()
                if self.controller.linear_e == 0 and self.controller.angular_e == 0:
                    self.state = 'find station'
                    self.stop()
                    time.sleep(1)


            self.start_time = self.get_clock().now()
            self.lidar_flag = False
        
        

def main():
    rclpy.init()
    to_goal_node = ToGoalNode()
    rclpy.spin(to_goal_node)
    to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

