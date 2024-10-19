#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node { 
    public:
        VideoPublisher(std::string topic_name, int device, int time_cycle);     // Constructor
        ~VideoPublisher();                                          // Destructor
        // Methods
        void createPublisher();     // Method to create the publisher
        void publishVideo();        // Method to publish the video
        void sendImage();           // Method to send the image
        void autoQoS();             // Method to set the QoS profile

    private:
        // Attributes
        std::string topic_name;              // Name of the topic (Is where the video will be published)
        int device;                          // Device to capture the video
        int time_cycle;                      // Time cycle to callback
        // Status
        bool working;                        // Status of the video
        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
        // Timer
        rclcpp::TimerBase::SharedPtr timer;
        // VideoCapture
        cv::VideoCapture capture;
        // QoS
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
};