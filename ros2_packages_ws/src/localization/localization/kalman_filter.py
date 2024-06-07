import rclpy
from rclpy.node import Node
import numpy as np
import math

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class KalmanFilter():
    def __init__(self):

        #parameters
        self.r = 0.5
        self.l = 0.19
        self.wr = 0.0
        self.wl = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity  = 0.0
        self.dt = 0.1
        self.marker_detected = None
        self.delta_x = 1.0
        self.delta_y = 1.0
        self.distance_p = 1.0

        self.robot_position = np.matrix([[0.0], 
                                           [0.0], 
                                           [0.0]])
        
        self.prev_robot_position = np.matrix([[0.0], 
                                        [0.0], 
                                        [0.0]])
        
        self.covariance_matrix = np.matrix([[0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0]])
        
        
        self.prev_covariance_matrix = np.matrix([[0.0, 0.0, 0.0],
                                                 [0.0, 0.0, 0.0],
                                                 [0.0, 0.0, 0.0]])
        
        self.aruco_position_1 = np.matrix([[1.34],
                                            [1.16]])
        
        self.aruco_position_2 = np.matrix([[0.0],
                                            [0.0]])
        
        self.motion_model_covariance = np.matrix([[0.0, 0.0, 0.0],
                                                [0.0, 0.0, 0.0],
                                                [0.0, 0.0, 0.0]])
        
        #z(zx, zy)
        self.lidar_coordinates = np.matrix([[0.0], 
                                            [0.0]])

        self.estimated_position = np.matrix([[0.0], 
                                           [0.0], 
                                           [0.0]])
        
        self.linearized_model =  np.matrix([[1.0, 0.0, 0.0],
                                           [0.0, 1.0, 0.0],
                                           [0.0, 0.0, 1.0]])
        
        self.propagation_uncertainty = np.matrix([[0.0, 0.0, 0.0],
                                                  [0.0, 0.0, 0.0],
                                                  [0.0, 0.0, 0.0]])
        
        self.observation_model = np.matrix([[0.0],
                                            [0.0]])
        
        self.linearise_observation_model = np.matrix([[0.0, 0.0, 0.0],
                                                      [0.0, 0.0, -1.0]])
        
        self.measurement_uncertainty = np.matrix([[0.0, 0.0],
                                                  [0.0, 0.0]])
        
        self.kalman_gain = np.matrix([[0.0, 0.0],
                                      [0.0, 0.0],
                                      [0.0, 0.0]])
        
        self.position_kalman_filter = np.array([[0.0],
                                                [0.0],
                                                [0.0]])
        
        #constants
        self.observation_model_covariance = np.matrix([[0.02, 0.0],
                                                       [0.0, 0.05]])
        
        self.identity_matrix = np.matrix([[1.0, 0.0, 0.0],
                                          [0.0, 1.0, 0.0],
                                          [0.0, 0.0, 1.0]])
        

        #messages
        self.msg_odom = Odometry()  
        self.msg_lidar = LaserScan()      


    def kalman_filter(self):
        #update linear and angular velocities

        if(self.marker_detected == 7):
            #delta x, delta y, distance p
            self.delta_x = self.aruco_position_1[0,0] - self.robot_position[0,0]
            self.delta_y = self.aruco_position_1[1,0] - self.robot_position[1,0]
            self.distance_p = self.delta_x ** 2 + self.delta_y ** 2
        elif(self.marker_detected == 8):
            #delta x, delta y, distance p
            self.delta_x = self.aruco_position_2[0,0] - self.robot_position[0,0]
            self.delta_y = self.aruco_position_2[1,0] - self.robot_position[1,0]
            self.distance_p = self.delta_x ** 2 + self.delta_y ** 2
       
        #observation model
        self.observation_model = np.matrix([[np.sqrt(self.distance_p)],
                                            [math.atan2(self.delta_y,self.delta_x) - self.theta]])
        
        

        
        #linearise observation model
        self.linearise_observation_model = np.matrix([[ -self.delta_x / np.sqrt(self.distance_p), -self.delta_y / np.sqrt(self.distance_p),  0],
                                                      [  self.delta_y / self.distance_p         , -self.delta_x / self.distance_p         , -1]])
        mult1 = self.linearise_observation_model*self.propagation_uncertainty
        mult2 = mult1 * self.linearise_observation_model.T
        #measurement uncertainty
        self.measurement_uncertainty = mult2 + self.observation_model_covariance

        #kalman gain
        self.kalman_gain = ((self.propagation_uncertainty*self.linearise_observation_model.T)*self.measurement_uncertainty.getI())
        # print("propagation: ", self.propagation_uncertainty)
        # print("linear transpose: ", self.linearise_observation_model.T)
        # print("measurement inverse: ", self.measurement_uncertainty.getI())
        # print("gain: ", self.kalman_gain)

        #position with kalman filter
        self.position_kalman_filter = self.estimated_position + (self.kalman_gain * (self.lidar_coordinates - self.observation_model))
        # print("Estimated pos: ", self.estimated_position)
        # print("Resta:" , self.lidar_coordinates - self.observation_model)

        # self.prev_position = self.initial_position
        self.prev_covariance_matrix = self.covariance_matrix

        #new covariance matrix
        self.covariance_matrix = ((self.identity_matrix - (self.kalman_gain * self.linearise_observation_model))* self.propagation_uncertainty)
        

