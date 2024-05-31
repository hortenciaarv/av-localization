import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection



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


class FramePublisher(Node):

    def __init__(self):
        super().__init__('coordinate_transform_publisher')


        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_puzzlebot_pose,
            1)
        self.subscription  # prevent unused variable warning

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.handle_scan,
            1)
        self.scan_subscription  # prevent unused variable warning

        self.aruco_subscription = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.handle_aruco,
            1)
        self.aruco_subscription  # prevent unused variable warning


    def handle_puzzlebot_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = quaternion_from_euler(0, 0, msg.pose.pose.orientation.w )
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def handle_scan(self, msg):
        t = TransformStamped()

        # Set the header and frame IDs
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'

        # Assuming the laser is mounted at the origin of base_link, with no offset
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Assuming the laser is mounted with no rotation relative to base_link
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def handle_aruco(self, msg):

        # Assuming the laser is mounted at the origin of base_link, with no offset
        for i in msg.markers:
            t = TransformStamped()

            # Set the header and frame IDs
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'marker_' + str(i.marker_id)
            t.transform.translation.x = i.pose.position.z
            t.transform.translation.y = -i.pose.position.x
            t.transform.translation.z = -i.pose.position.y

            # Assuming the laser is mounted with no rotation relative to base_link
            q = quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

        
def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()