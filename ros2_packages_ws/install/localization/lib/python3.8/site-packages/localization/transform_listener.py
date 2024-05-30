import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


from turtlesim.srv import Spawn
import sys
sys.path.append('/home/puzzlebot/ros2_packages_ws/src/localization/localization/')
from class_controller import Controller

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
     
        return yaw_z # in radians


class FrameListener(Node):

    def __init__(self):
        super().__init__('ArucoFrameListener')

        self.controller = Controller()
        self.controller.angular_error_min = 0.35
        self.controller.linear_error_min = 0.2
        self.controller.kp_l = 0.7
        self.controller.kp_w = 1.6

        self.cmd_vel_msg = Twist()

        self.x_pose = 0.0
        self.z_pose = 0.0
        self.theta_pose = 0.0
        self.x_d = 0.0
        self.z_d = 0.0
        self.theta_d = 0.0

        self.linear_e = 0
        self.prev_linear_e = 0
        self.angular_e = 0
        self.prev_angular_e = 0
        self.integral_e_linear = 0
        self.integral_e_angular = 0
        self.x_e = 0
        self.y_e = 0


        self.kp_l = 0.6
        self.ki_l = 0.0
        self.kd_l = 0.0

        self.kp_w = 1.05
        self.ki_w = 0
        self.kd_w = 0

        print("Running transform listener")

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'marker_6').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Create publisher and suscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom',self.pose_update,1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)



        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
    
    def pose_update(self, msg):
        self.controller.x_pose = msg.pose.pose.position.x
        self.controller.y_pose = msg.pose.pose.position.y
        self.controller.theta_pose = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'odom'

        msg = Twist()
        
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            

            self.controller.x_d = t.transform.translation.x
            self.controller.y_d = t.transform.translation.y


            self.controller.controller()
            print("Error lineal: ", self.controller.linear_e)
            print("Error angular: ", self.controller.angular_e)

            msg.linear.x = self.controller.cmd_vel_msg.linear.x
            msg.angular.z = self.controller.cmd_vel_msg.angular.z

            self.pub_cmd_vel.publish(msg)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.pub_cmd_vel.publish(msg)

            return

       
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()