from std_msgs.msg import Bool
from std_msgs.msg import Int32

class gripper():
    def __init__(self):

        self.servo_angle = Int32() 
        self.gripper_status = False
        self.gpio_value = Bool()

    def close_gripper(self):
        self.servo_angle = 50

    def open_gripper(self):
        self.servo_angle = 120

    def get_gripper_status(self):
        if(self.gpio_value):
            print("Esta presionado")
        else:
            print("No esta presionado")

