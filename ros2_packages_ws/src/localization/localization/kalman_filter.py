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
        self.linear = 0.0
        self.angular  = 0.0
        self.dt = 0.1

        self.initial_position = np.matrix([[0], 
                                           [0], 
                                           [0]])
        
        self.prev_position = np.matrix([[0], 
                                        [0], 
                                        [0]])
        
        self.covariance_matrix = np.matrix([[0, 0, 0],
                                            [0, 0, 0],
                                            [0, 0, 0]])
        
        
        self.prev_covariance_matrix = np.matrix([[0, 0, 0],
                                                 [0, 0, 0],
                                                 [0, 0, 0]])
        
        self.landmark_position = np.matrix([[0],
                                            [0]])
        
        #z(zx, zy)
        self.lidar_coordinates = np.matrix([[0], 
                                            [0]])

        self.estimated_position = np.matrix([[0], 
                                           [0], 
                                           [0]])
        
        self.linearized_model =  np.matrix([[1, 0, 0],
                                           [0, 1, 0],
                                           [0, 0, 1]])
        
        self.propagation_uncertainty = np.matrix([[0, 0, 0],
                                                  [0, 0, 0],
                                                  [0, 0, 0]])
        
        self.observation_model = np.matrix([[0],
                                            [0]])
        
        self.linearise_observation_model = np.matrix([[0, 0, 0],
                                                      [0, 0, -1]])
        
        self.measurement_uncertainty = np.matrix([[0, 0],
                                                  [0, 0]])
        
        self.kalman_gain = np.matrix([[0, 0],
                                      [0, 0]])
        
        self.position_kalman_filter = np.array([[0],
                                                [0],
                                                [0]])
        
        #constants
        self.motion_model_covariance = np.matrix([[0.5, 0.01, 0.01],
                                                  [0.01, 0.5, 0.01],
                                                  [0.01, 0.01, 0.2]])
        
        self.observation_model_covariance = np.matrix([[0.01, 0],
                                                       [0, 0.01]])
        
        self.identity_matrix = np.matrix([[1, 0, 0],
                                          [0, 1, 0],
                                          [0, 0, 1]])
        

        #messages
        self.msg_odom = Odometry()  
        self.msg_lidar = LaserScan()      


    def kalman_filter(self):
        #update linear and angular velocities
        # self.update_linear()
        # self.update_angular()

        #estimated position
        # matrix_A = np.matrix([[self.dt * self.linear * np.cos(self.prev_position[2,0])],
        #                       [self.dt * self.linear * np.sin(self.prev_position[2,0])],
        #                       [self.dt * self.angular]])
        
        # self.estimated_position = self.prev_position + matrix_A

        #linearized model
        # self.linearized_model =  np.matrix([[1, 0, -self.dt * self.linear * np.sin(self.prev_position[2,0])],
        #                                     [0, 1,  self.dt * self.linear * np.cos(self.prev_position[2,0])],
        #                                     [0, 0,  1]])
        
        #propagation of the uncertainity
        # self.propagation_uncertainty = np.multiply(np.multiply(self.linearized_model, self.prev_covariance_matrix), self.linearized_model.T) + self.motion_model_covariance

        #delta x, delta y, distance p
        delta_x = self.landmark_position[0,0] - self.initial_position[0,0]
        delta_y = self.landmark_position[1,0] - self.initial_position[1,0]
        distance_p = delta_x ** 2 + delta_y ** 2

        #observation model
        self.observation_model = np.matrix([[np.sqrt(distance_p)],
                                            [math.atan2(delta_y,delta_x)]])
        
        #linearise observation model
        self.linearise_observation_model = np.matrix([[- delta_x / np.sqrt(distance_p), - delta_y / np.sqrt(distance_p),  0],
                                                      [  delta_y / distance_p         , - delta_x / distance_p         , -1]])
        mult1 = self.linearise_observation_model*self.propagation_uncertainty
        mult2 = mult1 * self.linearise_observation_model.T
        #measurement uncertainty
        self.measurement_uncertainty = mult2 + self.observation_model_covariance

        #kalman gain
        self.kalman_gain = ((self.propagation_uncertainty*self.linearise_observation_model.T)*self.measurement_uncertainty.getI())

        #position with kalman filter
        self.position_kalman_filter = self.estimated_position + (self.kalman_gain * (self.lidar_coordinates - self.observation_model))

        # self.prev_position = self.initial_position
        self.prev_covariance_matrix = self.covariance_matrix

        #new covariance matrix
        self.covariance_matrix = ((self.identity_matrix - (self.kalman_gain * self.linearise_observation_model))* self.propagation_uncertainty)
        
# def main(args=None):
#     rclpy.init(args=args)

#     kalman = KalmanFilter()

#     rclpy.spin(kalman)

#     # Destroy the node 
#     kalman.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





