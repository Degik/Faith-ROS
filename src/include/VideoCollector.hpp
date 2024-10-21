#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoCollector : public rclcpp::Node { 
    public:
        VideoCollector(std::string topic_name);     // Constructor
        ~VideoCollector();                            // Destructor
        // Methods
        void createSubscription();     // Method to create the Subscription

};