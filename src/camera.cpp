#include <iostream>
#include <VideoPublisher.hpp>

// These node may capture the output video and sending them to the master node
int main(void){
    rclcpp::init(0, nullptr); // Initialize the ROS2
    // Create the publisher
    VideoPublisher video_publisher("camera", 0, 100); // Topic name: camera, Device: 0, Time cycle: 100
    video_publisher.createPublisher();
    video_publisher.publishVideo();
    // Spin the node
    rclcpp::spin(video_publisher.get_node_base_interface());
    // Exit the node
    rclcpp::shutdown();
    return 0;
}