import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import sys
import json

class Sequence_time_node(Node):
    def __init__(self):
        super().__init__('sequence_time_node')

        # Si se quiere trabajar con opencv
        self.bridge = CvBridge()

        self.camera_360 = self.create_subscription(
            CompressedImage,
            '/camera_360/image/compressed',  
            self.listener_camera_360_callback,
            10
        )
        self.zed = self.create_subscription(
            CompressedImage,
            '/zed/zed_node/rgb/image_rect_color/compressed',  
            self.listener_zed_callback,
            10
        )

        self.sync = self.create_subscription(
            String,
            '/sync_command',
            self.listener_sync_callback,
            10)
        
        self.clock = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.listener_clock_callback,
            10)

        self.last_msg_time = self.get_clock().now()
        self.timeout_seconds = 60.0  # Tiempo de espera m�ximo sin recibir mensajes
        self.timer = self.create_timer(1.0, self.check_topic_timeout)
        self.exit = False
        self.start_analysis = False

        self.currentTime = 0
        self.total_time = 0
        self.total_effective_time = 0
        self.start_time = 0
        self.initial_time = -1
        self.started = False
        self.discarded = False
        self.trajectories = list()
        self.new_trajectory = dict()
        self.nImgs360 = 0
        self.nImgsZed = 0
        self.get_logger().info('Nodo listo para analizar la duración de las secuencias')

    def check_topic_timeout(self):
        now = self.get_clock().now()
        if self.start_analysis and (now - self.last_msg_time) > Duration(seconds=self.timeout_seconds):
            self.get_logger().warn('No se ha recibido ning�n mensaje en los �ltimos 5 segundos.')
            final_data = dict()
            final_data['trajectories'] = self.trajectories
            final_data['effective_time'] = self.total_effective_time
            with open('sequence_validity.json', 'w') as outfile: 
                json.dump(final_data, outfile, indent=4, sort_keys=True) 
                outfile.close()

            self.exit = True

    def listener_camera_360_callback(self, msg):
        self.start_analysis = True
        self.last_msg_time = self.get_clock().now()


        self.nImgs360 += 1


    def listener_zed_callback(self, msg):
        self.start_analysis = True
        self.last_msg_time = self.get_clock().now()

        self.nImgsZed += 1        


    def listener_sync_callback(self, msg):
        self.start_analysis = True
        self.last_msg_time = self.get_clock().now()
        self.get_logger().info(f'Recibí comando de sincronización')
        print('sincronización', msg, self.currentTime)
        if msg.data == 'start':
            self.start_time = self.current_time
            self.started = True
            self.discarded = False
            self.nImgs360 = 0
            self.nImgsZed = 0
            self.new_trajectory = dict()
        if msg.data == 'goal':
            trajectory_length = self.current_time-self.start_time
            self.new_trajectory['time'] = trajectory_length
            self.new_trajectory['nImgs360'] = self.nImgs360
            self.new_trajectory['nImgsZed'] = self.nImgsZed
            self.trajectories.append(self.new_trajectory)
            self.total_effective_time += trajectory_length
            self.total_time = self.current_time - self.initial_time
            self.started = False
            self.discarded = False
            print(f'Total effective time: {self.total_effective_time} from {self.total_time}')
            for i_t, t in enumerate(self.trajectories):
                print(i_t, t)

        if msg.data == 'discard':
            self.started = False
            self.discarded = True
        

    def listener_clock_callback(self, msg):
        self.start_analysis = True
        self.last_msg_time = self.get_clock().now()

        self.current_time = msg.header.stamp.sec
        if self.initial_time < 0:
            self.initial_time = self.current_time


def main(args=None):
    rclpy.init(args=args)
    node = Sequence_time_node()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.exit:
            break

    print('destroy')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

