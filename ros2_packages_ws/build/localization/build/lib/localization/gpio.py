import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class gpio_read(Node):
    def __init__(self):
        super().__init__("gpio_read_node")

        self.channel = 10 #pin number
        self.gpio_value_msg = Bool()

        self.gpio_value_pub = self.create_publisher(Bool, '/gpio_value', 10)

        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.read_gpio)


    def read_gpio(self):

        #Establish connection with the channel    
        GPIO.setmode(GPIO, self.channel) 

        #Set GPIO as input
        GPIO.setup(self.channel, GPIO.IN)

        #Read GPIO value
        gpio_value = GPIO.input(self.channel)

        if (gpio_value == 'HIGH'):
            self.gpio_value_msg = True
        else:
            self.gpio_value_msg = False

        self.gpio_value_pub.publish(self.gpio_value_msg)

        GPIO.cleanup()

def main():
    rclpy.init()
    gpio_value_node = gpio_read()
    rclpy.spin(gpio_value_node)
    gpio_value_node.destroy_node()
    rclpy.shutdown()
