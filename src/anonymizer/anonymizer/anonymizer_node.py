import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Anonymizer_node(Node):
    def __init__(self):
        super().__init__('anonymizer_node')

        # Si se quiere trabajar con opencv
        self.bridge = CvBridge()

        # Suscripciones a los topics de la cámaras
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
        
        # Publicación de imagen anonimizada
        self.publisher_camera_360 = self.create_publisher(Image, 'camera_360/anonymizer/image', 10)
        self.compressed_publisher_camera_360 =  self.create_publisher(CompressedImage, '/camera_360/anonymizer/image/compressed', 10)
        # Publicación de imagen anonimizada
        self.publisher_zed = self.create_publisher(Image, '/zed/zed_node/rgb/anonymizer/image_rect_color', 10)
        self.compressed_publisher_zed =  self.create_publisher(CompressedImage, '/zed/zed_node/rgb/anonymizer/image_rect_color/compressed', 10)

        self.get_logger().info('Nodo listo para recibir y publicar imágenes')

    def listener_camera_360_callback(self, msg):
        self.get_logger().info(f'Recibí una imagen de Cámara 360')
        

        # Convertir la imagen de ROS a OpenCV
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        
        #####################INSERT YOUR CODE HEEEERRREEE###########################
        # Aquí puedes realizar procesamiento de la imagen si lo deseas, por ejemplo:
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Imagen Recibida Camara 360", cv_image)
        cv2.waitKey(1)


        ##################################################################################

        # Convertir la imagen procesada de OpenCV a ROS
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        compressed_image = cv2.imencode('.jpg', cv_image)[1].tobytes()
        compressed_msg = CompressedImage()
        compressed_msg.data = compressed_image

        # Publicar la imagen procesada
        self.publisher_camera_360.publish(msg)
        self.compressed_publisher_camera_360. publish(compressed_msg)
        self.get_logger().info('Imagen publicada')

    def listener_zed_callback(self, msg):
        self.get_logger().info(f'Recibí una imagen de Zed')
        

        # Convertir la imagen de ROS a OpenCV
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        
        #####################INSERT YOUR CODE HEEEERRREEE###########################
        # Aquí puedes realizar procesamiento de la imagen si lo deseas, por ejemplo:
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Imagen Recibida Zed", cv_image)
        cv2.waitKey(1)


        ##################################################################################

        # Convertir la imagen procesada de OpenCV a ROS
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        compressed_image = cv2.imencode('.jpg', cv_image)[1].tobytes()
        compressed_msg = CompressedImage()
        compressed_msg.data = compressed_image

        # Publicar la imagen procesada
        self.publisher_zed.publish(msg)
        self.compressed_publisher_zed. publish(compressed_msg)
        self.get_logger().info('Imagen publicada')

def main(args=None):
    rclpy.init(args=args)

    node = Anonymizer_node()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
