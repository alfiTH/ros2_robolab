#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG
#if defined(DEBUG)
    #include <chrono>
#endif // DEBUG


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

    ~RicoThetaZ1(){
        capture.release();
    }

private:
    void capture_and_publish()
    {
        // Verificar si la cámara está abierta
        try{
            if (!capture.isOpened())
            {
                capture.release();
                sleep(1);
                capture.open(pipeline, cv::CAP_GSTREAMER);
                if (!capture.isOpened())  // Intentar abrir la cámara solo una vez si no se puede abrir
                {
                    RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara");
                    return;
                }
            }
        }
        catch (const std::exception& ex){
            RCLCPP_ERROR(this->get_logger(), ex.what());
            return;
        }
        #if defined(DEBUG)
            auto t1 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG

        capture >> frame; // Captura el frame de la cámara

        #if defined(DEBUG)
            auto t2 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Frame vacío recibido");
            return;
        }


        #if defined(DEBUG)
            auto t3 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG
        cv::resize(frame, frame, this->frame_size);


        #if defined(DEBUG)
            auto t4 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG
        // Convertir el frame de OpenCV a un mensaje de imagen de ROS
        auto tmp_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);

        #if defined(DEBUG)
            auto t5 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG

        auto msg = tmp_msg.toImageMsg();
        #if defined(DEBUG)
            auto t6 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG
        auto compressed_msg = tmp_msg.toCompressedImageMsg(cv_bridge::JPG);
        #if defined(DEBUG)
            auto t7 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG

       

        // Publicar la imagen
        image_pub_->publish(*msg);
        #if defined(DEBUG)
            auto t8 = std::chrono::high_resolution_clock::now();
        #endif // DEBUG

        // Crear y publicar la imagen comprimida
        compressed_image_pub_->publish(*compressed_msg);

        #if defined(DEBUG)
            auto t9 = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> capture_duration = t2 - t1;
            std::chrono::duration<double> resize_duration = t4 - t3;
            std::chrono::duration<double> bridge_duration = t5 - t4;
            std::chrono::duration<double> msg_duration = t6 - t5;
            std::chrono::duration<double> compress_duration = t7 - t6;
            std::chrono::duration<double> pub_duration = t8 - t7;
            std::chrono::duration<double> pub_compress_duration = t9 - t8;
            RCLCPP_INFO(this->get_logger(), "Tiempo de captura:                         %f segundos", capture_duration.count());
            RCLCPP_INFO(this->get_logger(), "Tiempo de redimensionado:                  %f segundos", resize_duration.count());
            RCLCPP_INFO(this->get_logger(), "Tiempo de conversión a mensaje (cv_bridge):%f segundos", bridge_duration.count());
            RCLCPP_INFO(this->get_logger(), "Tiempo de creación del mensaje:            %f segundos", msg_duration.count());
            RCLCPP_INFO(this->get_logger(), "Tiempo de compresión:                      %f segundos", compress_duration.count());
            RCLCPP_INFO(this->get_logger(), "Tiempo de publicación:                     %f segundos", pub_duration.count());
            RCLCPP_INFO(this->get_logger(), "Tiempo de publicación de la compresión:    %f segundos", pub_compress_duration.count());

        #endif // DEBUG

    }


    //Opencv things
    cv::Mat frame;
    cv::VideoCapture capture;
    std::string pipeline;   
    const cv::Size frame_size = cv::Size(960, 480);

    // Ros2 things
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RicoThetaZ1>());
    rclcpp::shutdown();
    return 0;
}