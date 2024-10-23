#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

using vector_str = std::vector<std::string>;
using msg_ptr = const sensor_msgs::msg::Image::SharedPtr;


// The collector needs to implemented with a multithread approach!
class VideoCollector : public rclcpp::Node { 
    public:
        VideoCollector(vector_str camera_topics);     // Constructor
        ~VideoCollector();                            // Destructor
        // Methods
        void createSubscription();                 // Method to create the subscriptions list
        void takeFrame(int index, msg_ptr msg);    // This method take the frame and save it
        void createFolders();                      // This method create the folders
        void processFrames();                      // This method process the frames
    private:
        // Attributes
        vector_str camera_topics;
        rclcpp::TimerBase::SharedPtr timer;
        std::vector<cv::VideoWriter> video_writers;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions;
};