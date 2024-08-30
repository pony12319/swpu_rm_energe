#include "video_test/video_read_node.hpp"

namespace VideoPublic
{
    VideoPublicer::VideoPublicer(const rclcpp::NodeOptions &options) : Node("VideoPublisher", options)
    {
        RCLCPP_INFO(get_logger(), "Start VideoTest!");

        // 创建图像发布者
        bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
        auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
        image_publisher_ = image_transport::create_publisher(this,"/image_raw", qos);

        // 打开视频文件
        video_capture_ = cv::VideoCapture("src/video_test/video/SCU_blue2.avi");
        if (!video_capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
        }

        // 创建定时器，以固定频率发布图像
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        PublishImage();
        });
    }

    void VideoPublicer::PublishImage(void)
    {
        // 读取视频帧
        cv::Mat frame;
        video_capture_ >> frame;

        if (frame.empty()) {
        // 如果没有更多的帧，重置视频捕捉以循环播放
            video_capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            video_capture_ >> frame;
        }
        
        // 将 OpenCV 的 Mat 转换为 ROS 的 sensor_msgs::msg::Image
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        img_msg->header.stamp = this->now();
        auto time1 = this->now();
        // rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1000)));
        auto time2= this->now();
        //std::cout<<"the time is "<< time2.seconds() - time1.seconds() << std::endl;
        //std::cout<<"the time is "<< (time2-time1).seconds() << std::endl;
        // 发布图像消息
        image_publisher_.publish(*img_msg);
        // RCLCPP_INFO(this->get_logger(), "pub success");
    }
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(VideoPublic::VideoPublicer)

