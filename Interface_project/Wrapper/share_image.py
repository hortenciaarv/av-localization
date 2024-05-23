import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Pose

def publish_image(image_path):
    # Inicializar el nodo ROS
    rospy.init_node('image_publisher', anonymous=True)
    
    # Crear un objeto CvBridge
    bridge = CvBridge()
    
    # Crear un publicador para el tema "/image_raw"
    image_publisher = rospy.Publisher('/Image', Image, queue_size=10)
    pose_publisher = rospy.Publisher('/odom' , Pose, queue_size=10)
    
    # Frecuencia de publicación (en Hz)
    pose_msg = Pose()
    pose_msg.position.x = 0
    pose_msg.position.y = 0
    pose_msg.position.z = 0
    rate = rospy.Rate(1)  # Publicar una vez por segundo
    
    while not rospy.is_shutdown():
        try:
            # Leer la imagen desde el archivo
            cv_image = cv2.imread(image_path)
            
            # Convertir la imagen de OpenCV a un mensaje de sensor_msgs/Image
            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # Publicar la imagen
            image_publisher.publish(ros_image)
            pose_publisher.publish(pose_msg)

            pose_msg.position.x += 1
            pose_msg.position.y += 1
            pose_msg.position.z += 1
            
            rospy.loginfo("Image published")
        except Exception as e:
            rospy.logerr("Error while publishing image: %s", str(e))
        
        rate.sleep()

if __name__ == '__main__':
    image_path = "/home/robotics/work/ROS-Object-Detection/isorepublic-red-green-apples-1-1100x733.jpg"  # Ruta de la imagen que deseas publicar
    publish_image(image_path)