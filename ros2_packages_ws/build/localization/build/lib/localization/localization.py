#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection
import sys
sys.path.append('/home/puzzlebot/Puzzlebot_Lidar_ROS1_ROS2/ros2_packages_ws/src/localization/localization')
from kalman_filter import KalmanFilter

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class localizationPublisher(Node):
    def __init__(self):
        super().__init__('localization')
        self.l = 0.19
        self.r = 0.05
        self.x = 0
        self.x_prev = 0
        self.y = 0
        self.y_prev = 0
        self.theta = 0
        self.theta_prev = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_theta = 0
        self.v = 0
        self.w = 0
        self.wl = 0
        self.wr = 0
        self.kr = 100
        self.kl = 100
        self.k = (1/2)*self.r
        self.covariance_matrix = np.matrix([[0,0,0],[0,0,0],[0,0,0]])
        self.prev_covariance_matrix = np.matrix([[0,0,0],[0,0,0],[0,0,0]])
        self.complete_matrix = np.zeros(36)
        self.jacobian =  np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        self.error_matrix =  np.matrix([[0,0,0],[0,0,0],[0,0,0]])
        self.delta_k = np.matrix([[0,0],[0,0]])
        self.nabla_w = np.matrix([[0,0],[0,0],[0,0]])

        self.odom_msg = Odometry()
        self.kalman = KalmanFilter()
        self.kalman_filter_flag = Bool()
        self.kalman_filter_flag.data = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
        print("The localization node is Running")
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)
        self.wr_subscription = self.create_subscription(Float32, '/VelocityEncR', self.wr_update,
        rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.wl_subscription = self.create_subscription(Float32, '/VelocityEncL', self.wl_update,         
        rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.subscription_aruco = self.create_subscription(ArucoDetection, 'aruco_detections', self.cb_aruco, 10)
        
        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.update_odom)

    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def update_kalman_filter_variables(self):
        self.kalman.linear_velocity = self.v
        self.kalman.angular_velocity = self.w
        # self.msg_odom = msg
        self.kalman.theta = self.theta
        self.kalman.dt = self.dt
        self.kalman.motion_model_covariance = self.covariance_matrix
        self.kalman.prev_robot_position = np.matrix([[self.x_prev], 
                                        [self.y_prev], 
                                        [self.theta_prev]])

        self.kalman.robot_position = np.matrix([[self.x],
                                           [self.y],
                                           [self.theta]])
        
        matrix_A = np.matrix([[self.kalman.dt * self.kalman.linear_velocity * np.cos(self.kalman.prev_robot_position[2,0])],
                              [self.kalman.dt * self.kalman.linear_velocity * np.sin(self.kalman.prev_robot_position[2,0])],
                              [self.kalman.dt * self.kalman.angular_velocity]])
        
        self.kalman.estimated_position = self.kalman.prev_robot_position + matrix_A
        self.kalman.linearized_model =  np.matrix([[1, 0, -self.kalman.dt * self.kalman.linear_velocity * np.sin(self.kalman.prev_robot_position[2,0])],
                                            [0, 1,  self.kalman.dt * self.kalman.linear_velocity * np.cos(self.kalman.prev_robot_position[2,0])],
                                            [0, 0,  1]])
        self.kalman.propagation_uncertainty = np.multiply(np.multiply(self.kalman.linearized_model, self.kalman.prev_covariance_matrix), self.kalman.linearized_model.T) + self.kalman.motion_model_covariance
        print("covariance: ", self.covariance_matrix)
        # print("covariance kalman:", self.kalman.motion_model_covariance)


    def find_aruco(self):
        self.kalman.kalman_filter()
        # print("Hice kalman")
        self.covariance_matrix = self.kalman.covariance_matrix
        self.x = self.kalman.position_kalman_filter[0,0]
        self.y = self.kalman.position_kalman_filter[1,0]
        self.theta = self.kalman.position_kalman_filter[2,0]
        print("Kalman position ")
        print(self.x)
        print(self.y)
        print(self.theta)
        print(self.kalman.lidar_coordinates)
        self.kalman_filter_flag.data = False
        
    def update_odom(self):
        self.calculate_dt()
        self.transform()
        self.differential_drive_model()
        self.solver()
        self.calculate_covariance()

        # print(self.x)
        # print(self.y)
        # print(self.theta)

        self.update_kalman_filter_variables()
        if self.kalman_filter_flag.data:
            self.find_aruco()

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y

        # print(self.x)
        # print(self.y)

        q = quaternion_from_euler(0, 0, self.theta )
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x = self.v
        self.odom_msg.twist.twist.angular.z = self.vel_theta
        self.odom_msg.pose.covariance = self.complete_matrix
        self.odom_msg.twist.covariance = np.zeros(36)

        self.publisher_odom.publish(self.odom_msg)
        self.start_time = self.get_clock().now()
        self.x_prev = self.x
        self.y_prev = self.y
        self.theta_prev = self.theta
        self.prev_covariance_matrix = self.covariance_matrix

    def differential_drive_model(self):
        self.vel_x = self.v*np.cos(self.theta)
        self.vel_y = self.v*np.sin(self.theta)
        self.vel_theta =self.w

    def solver(self):
        self.x += self.vel_x*self.dt
        self.y += self.vel_y*self.dt
        self.theta_prev = self.theta
        self.theta += self.vel_theta*self.dt

    def transform(self):
        self.v = self.r*(self.wr + self.wl)/2
        self.w = self.r*(self.wr - self.wl)/self.l
    
        
    def cb_aruco(self, msg):
        for i in msg.markers:
            if (i.marker_id == 7 or i.marker_id == 8):
               self.kalman.marker_detected = i.marker_id
               self.kalman.lidar_coordinates[0,0] = np.sqrt(i.pose.position.z**2 + i.pose.position.x**2) + 0.1
               self.kalman.lidar_coordinates[1,0] = math.atan(i.pose.position.x / i.pose.position.z) + self.theta    
               self.kalman_filter_flag.data = True
                
       
    def wr_update(self,msg):
        self.wr = msg.data    

    def wl_update(self,msg):
        self.wl = msg.data


    def calculate_nabla_w(self):
        matrix = np.matrix([[np.cos(self.theta_prev),np.cos(self.theta_prev)],
                          [np.sin(self.theta_prev),np.sin(self.theta_prev)],
                          [2/self.l,-2/self.l]])
        self.nabla_w = self.k*self.dt*matrix

    def calculate_delta_k(self):
        self.delta_k[0,0]= self.kr*np.abs(self.wr)
        self.delta_k[1,1]= self.kl*np.abs(self.wl)

    def calculate_error_matrix(self):
        self.calculate_nabla_w()
        self.calculate_delta_k()
        self.error_matrix = self.nabla_w*self.delta_k*np.transpose(self.nabla_w)

    def calculate_jacobian(self):
        self.jacobian[0,2] = -1*self.dt*self.v*np.sin(self.theta_prev)
        self.jacobian[1,2] = self.dt*self.v*np.cos(self.theta_prev)

    def calculate_covariance(self):
        self.calculate_jacobian()
        self.calculate_error_matrix()
        self.covariance_matrix = self.jacobian*self.covariance_matrix*np.transpose(self.jacobian) + self.error_matrix
        self.complete_matrix[0] = self.covariance_matrix[0,0]
        self.complete_matrix[1] = self.covariance_matrix[0,1]
        self.complete_matrix[5] = self.covariance_matrix[0,2]
        self.complete_matrix[6] = self.covariance_matrix[1,0]
        self.complete_matrix[7] = self.covariance_matrix[1,1]
        self.complete_matrix[11] = self.covariance_matrix[1,2]
        self.complete_matrix[30] = self.covariance_matrix[2,0]
        self.complete_matrix[31] = self.covariance_matrix[2,1]
        self.complete_matrix[35] = self.covariance_matrix[2,2]


def main():
    rclpy.init()
    localization_node = localizationPublisher()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()
