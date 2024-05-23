import grpc
import wrapper_pb2
import wrapper_pb2_grpc
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher("/image_from_grpc", Image, queue_size=10)
        rospy.init_node('grpc_image_publisher', anonymous=True)
        self.channel = grpc.insecure_channel('localhost:50051')  # Dirección del servidor gRPC
        self.stub = wrapper_pb2_grpc.WrapperStub(self.channel)

    def run(self):
        # Mantenerse suscrito al servidor para recibir imágenes
        response = self.stub.UploadImage(wrapper_pb2.Empty())
        # Convertir los datos de la imagen recibida a un mensaje de sensor_msgs/Image
        if response.data:
            # Convertir los datos de la imagen recibida a un arreglo de NumPy
            np_arr = np.frombuffer(response.data, dtype=np.uint8)
            # Decodificar la imagen en un formato que OpenCV pueda manejar
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image_np is not None:
                # Convertir el arreglo NumPy a un mensaje de sensor_msgs/Image
                image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
                # Publicar la imagen en el tema de ROS
                self.image_publisher.publish(image_msg)
                rospy.loginfo("Image published to RViz")
            else:
                rospy.logwarn("Received image data could not be decoded into a valid image.")
        else:
            rospy.logwarn("No data received from gRPC server.")
        # image_msg = self.bridge.cv2_to_imgmsg(np.array(response.data), encoding="passthrough")
        # Publicar la imagen en el tema de ROS
        # self.image_publisher.publish(image_msg)
        # rospy.loginfo("Image published to RViz")

            

if __name__ == "__main__":
    try:
        image_subscriber = ImageSubscriber()
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass
