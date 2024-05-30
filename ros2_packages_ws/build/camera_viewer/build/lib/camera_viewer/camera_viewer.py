# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.sensor_id=0
        self.capture_width=1920
        self.capture_height=1080
        self.display_width=960
        self.display_height=540
        self.framerate=30
        self.flip_method=2
        self.bridge = CvBridge()
    
        print("Camera Viewer Node is running")
        self.pub_camera = self.create_publisher(Image, '/camera1/image_raw', 1)
        self.start_time = self.get_clock().now()
        timer_period = 1/10
        self.timer = self.create_timer(timer_period, self.show_camera)



    def gstreamer_pipeline(self):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                self.sensor_id,
                self.capture_width,
                self.capture_height,
                self.framerate,
                self.flip_method,
                self.display_width,
                self.display_height,   
            )
        )

    # def update_camera(self):
    #     self.show_camera()

    def show_camera(self):
        window_title = "CSI Camera"

        # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
        print(self.gstreamer_pipeline())
        video_capture = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if video_capture.isOpened():
            try:
                # window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
                while True:
                    ret_val, frame = video_capture.read()
                    #encoding="passthrough" mono8
                    if (ret_val):
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        # frame = cv2.resize(frame,(300,150))
                        # frame = cv2.GaussianBlur(frame, (9,9), 0)
                        imgMsg = self.bridge.cv2_to_imgmsg(frame, "mono8")
                        imgMsg.header.frame_id = "Imagen"
                        imgMsg.header.stamp = self.get_clock().now().to_msg()
                        self.pub_camera.publish(imgMsg)
                    # Check to see if the user closed the window
                    # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                    # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                    # if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    #     cv2.imshow(window_title, frame)
                    # else:
                    #     break 
                    keyCode = cv2.waitKey(10) & 0xFF
                    # Stop the program on the ESC key or 'q'
                    if keyCode == 27 or keyCode == ord('q'):
                        break
            finally:
                video_capture.release()
                cv2.destroyAllWindows()
        else:
            print("Error: Unable to open camera")


def main():
    rclpy.init()
    camera_node = CameraViewer()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()