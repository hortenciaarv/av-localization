import rclpy
from rclpy.node import Node
import numpy as np
import math

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        print("The Kalman filter node is Running")

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
        
        self.observation_model_covariance = np.matrix([[0.1, 0],
                                                       [0, 0.02]])
        
        self.identity_matrix = np.matrix([[1, 0, 0],
                                          [0, 1, 0],
                                          [0, 0, 1]])
        

        #messages
        self.msg_odom = Odometry()  
        self.msg_lidar = LaserScan()      

        #publishers
        #self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        #subscribers
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.subscription_wl = self.create_subscription(Float32, 'wl', self.cb_wl, 10)
        self.subscription_wr = self.create_subscription(Float32, 'wr', self.cb_wr, 10)    
        self.subscription_lidar = self.create_subscription(LaserScan, 'scan', self.cb_lidar, 10)   
        self.subscription_aruco = self.create_subscription(ArucoDetection, 'aruco_detections', self.cb_aruco, 10)

        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.kalman_filter)

    def cb_odom(self, msg):
        self.msg_odom = msg

        self.theta = self.euler_from_quaternion(self.msg_odom.pose.pose.orientation.x,
                                           self.msg_odom.pose.pose.orientation.y,
                                           self.msg_odom.pose.pose.orientation.z,
                                           self.msg_odom.pose.pose.orientation.w)

        self.initial_position = np.matrix([[self.msg_odom.pose.pose.position.x],
                                           [self.msg_odom.pose.pose.position.y],
                                           [self.theta]])
        
        self.covariance_matrix = np.matrix([[self.msg_odom.pose.covariance[0] , self.msg_odom.pose.covariance[1] , self.msg_odom.pose.covariance[5]],
                                            [self.msg_odom.pose.covariance[6] , self.msg_odom.pose.covariance[7] , self.msg_odom.pose.covariance[11]],
                                            [self.msg_odom.pose.covariance[30], self.msg_odom.pose.covariance[31], self.msg_odom.pose.covariance[35]]])

    def cb_wl(self, msg):
        self.wl = msg.data

    def cb_wr(self, msg):
        self.wr = msg.data

    def cb_lidar(self, msg):
        self.msg_lidar = msg
        zx = self.msg_lidar.ranges[0] * np.cos(self.theta)
        zy = self.msg_lidar.ranges[0] * np.sin(self.theta)

        self.lidar_coordinates = np.matrix([[zx],
                                            [zy]])
        
    def cb_aruco(self, msg):
        for i in msg.markers:
            if(i.marker_id == 7): 
                
                self.landmark_position = np.matrix([[i.pose.position.x],
                                                    [i.pose.position.z]])

    def euler_from_quaternion(x, y, z, w):
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
     
        #radians
        return yaw_z

    def update_linear(self):
        self.linear = self.r * (self.wr + self.wl) / 2

    def update_angular(self):
        self.angular = self.r * (self.wr - self.wl) / self.l

    def kalman_filter(self):
        #update linear and angular velocities
        self.update_linear()
        self.update_angular()

        #estimated position
        matrix_A = np.matrix([[self.dt * self.linear * np.cos(self.prev_position[2,0])],
                              [self.dt * self.linear * np.sin(self.prev_position[2,0])],
                              [self.dt * self.angular]])
        
        self.estimated_position = self.prev_position + matrix_A

        #linearized model
        self.linearized_model =  np.matrix([[1, 0, -self.dt * self.linear * np.sin(self.prev_position[2,0])],
                                            [0, 1,  self.dt * self.linear * np.cos(self.prev_position[2,0])],
                                            [0, 0,  1]])
        
        #propagation of the uncertainity
        self.propagation_uncertainty = np.multiply(np.multiply(self.linearized_model, self.prev_covariance_matrix), np.transpose(self.linearized_model)) + self.motion_model_covariance

        #delta x, delta y, distance p
        delta_x = self.landmark_position[0,0] - self.initial_position[0,0]
        delta_y = self.landmark_position[0,1] - self.initial_position[0,1]
        distance_p = delta_x ** 2 + delta_y ** 2

        #observation model
        self.observation_model = np.matrix([[np.sqrt(distance_p)],
                                            [math.atan2(delta_y,delta_x)]])
        
        #linearise observation model
        self.linearise_observation_model = np.matrix([[- delta_x / np.sqrt(distance_p), - delta_y / np.sqrt(distance_p),  0],
                                                      [  delta_y / distance_p         , - delta_x / distance_p         , -1]])
        
        #measurement uncertainty
        self.measurement_uncertainty = np.multiply(np.multiply(self.linearise_observation_model, self.observation_model), np.transpose(self.linearise_observation_model)) + self.observation_model_covariance

        #kalman gain
        self.kalman_gain = np.multiply(np.multiply(self.observation_model, np.transpose(self.linearise_observation_model)), self.measurement_uncertainty.getI())

        #position with kalman filter
        self.position_kalman_filter = self.estimated_position + np.multiply(self.kalman_gain, self.lidar_coordinates - self.observation_model)

        self.prev_position = self.initial_position
        self.prev_covariance_matrix = self.covariance_matrix

        #new covariance matrix
        self.covariance_matrix = self.multiply(self.identity_matrix - np.multiply(self.kalman_gain, self.linearise_observation_model), self.observation_model)

        
def main(args=None):
    rclpy.init(args=args)

    kalman = KalmanFilter()

    rclpy.spin(kalman)

    # Destroy the node 
    kalman.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





