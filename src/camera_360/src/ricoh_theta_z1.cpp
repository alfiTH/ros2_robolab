#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class RicoThetaZ1 : public rclcpp::Node
{
public:
    RicoThetaZ1() : Node("Rico_theta_z1")
    {
        // Creamos un publicador de imágenes
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera_360/image", 10);
        compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera_360/image/compressed", 10);

        // Inicializar el pipeline de GStreamer
        pipeline = "thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! gldownload ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false";
        capture.open(pipeline, cv::CAP_GSTREAMER);

        while (!capture.isOpened())
        {
            sleep(5);
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el pipeline de GStreamer, reintentando");
        }

        // Empezamos el bucle de captura de imágenes
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), std::bind(&RicoThetaZ1::capture_and_publish, this));
    }

private:
    void capture_and_publish()
    {
        // Verificar si la cámara está abierta
        if (!capture.isOpened())
        {
            capture.open(pipeline, cv::CAP_GSTREAMER);
            if (!capture.isOpened())  // Intentar abrir la cámara solo una vez si no se puede abrir
            {
                RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara");
                return;
            }
        }

        capture >> frame; // Captura el frame de la cámara

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Frame vacío recibido");
            return;
        }

        // Si la imagen es demasiado grande o se requiere un tamaño constante, redimensionamos solo cuando sea necesario
        // El factor de escala se establece al principio para evitar que se calcule en cada ciclo
        static const cv::Size new_size(frame.cols / 2, frame.rows / 2);
        if (frame.size() != new_size) {
            cv::resize(frame, frame, new_size);
        }

        // Convertir el frame de OpenCV a un mensaje de imagen de ROS
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publicar la imagen
        image_pub_->publish(*msg);

        // Crear y publicar la imagen comprimida
        auto compressed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg(cv_bridge::JPG);
        compressed_image_pub_->publish(*compressed_msg);
    }

    cv::Mat frame;
    cv::VideoCapture capture;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string pipeline;    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RicoThetaZ1>());
    rclcpp::shutdown();
    return 0;
}