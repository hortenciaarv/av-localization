
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose

import grpc

from concurrent import futures
import cv2

# import the generated classes
import wrapper_pb2
import wrapper_pb2_grpc
import base64
import numpy

class WrapperServicer(wrapper_pb2_grpc.WrapperServicer):

    def __init__ (self):
        self.data = None
        self.position = [0,0,0]
        rospy.Subscriber('/Image', Image, self.Image_callback ,queue_size = 10)
        rospy.Subscriber('/odom', Pose, self.Pose_callback ,queue_size = 10)


    def UploadImage(self, request, context):

        image_message = wrapper_pb2.Image(data=self.data)


        return image_message

    def UploadPose(self, request, context):
    
        # Crear el mensaje de la posición y asignar el mensaje de la imagen
        position_message = wrapper_pb2.Position(
            x=self.position[0],
            y=self.position[1],
            theta=self.position[2],
        )


        return position_message

    def Image_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Convertir la imagen OpenCV a bytes para enviarla
        image_compressed = numpy.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()

        self.data = image_compressed

    def Pose_callback(self, msg):
        
        self.position[0] = msg.position.x
        self.position[1] = msg.position.y
        self.position[2] = msg.position.z

if __name__ == '__main__':
    
    rospy.init_node('wrapper',anonymous = True)

    # create a gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    # use the generated function `add_CalculatorServicer_to_server`
    # to add the defined class to the server
    wrapper_pb2_grpc.add_WrapperServicer_to_server(
            WrapperServicer(), server)

    # listen on port 50051
    print('Starting server. Listening on port 50051.')
    server.add_insecure_port('[::]:50051')
    server.start()

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except KeyboardInterrupt:
        server.stop(0)