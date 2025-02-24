import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import SVD48V
import numpy as np
import time

class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')

        #variables de clase
        self.targetSpeed = np.array([[0.0], [0.0], [0.0]])
        self.oldTargetSpeed = np.array([[0.0], [0.0], [0.0]])
        self.driver=None
        self.joystickControl = False
        self.time_disble = time.time()
        self.time_emergency = time.time()
        self.time_move = time.time()

        # Suscribirse a los topics
        self.subscription_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.subscription_nav = self.create_subscription(
            Twist, '/cmd_vel', self.nav_callback, 10)

    def joy_callback(self, msg):
        """ Callback del joystick """
        twist = Twist()
        twist.linear.x = msg.axes[1]  # Eje Y del joystick
        twist.angular.z = msg.axes[0]  # Eje X del joystick
        self.joy_cmd = twist

    def nav_callback(self, msg):
        """ Callback del stack de navegación """
        self.nav_cmd = msg

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    rclpy.spin(node)
    print("Finalizando base")
    node.driver.__del__()
    print("Base destruida")    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
