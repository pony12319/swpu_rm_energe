#ifndef _VIDEO_READ_NODE_HPP_
#define _VIDEO_READ_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <memory>
#include <chrono>

namespace VideoPublic
{
    class VideoPublicer : public rclcpp::Node
    {
        public:
            VideoPublicer(const rclcpp::NodeOptions &options);

        private:
            void PublishImage();

            rclcpp::TimerBase::SharedPtr timer_;
            image_transport::Publisher image_publisher_;
            cv::VideoCapture video_capture_;
            int count_;
    };

}

#endif