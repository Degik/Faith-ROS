#include <iostream>
#include <VideoPublisher.hpp>

// These node may capture the output video and sending them to the master node
int main(void){
    rclcpp::init(0, nullptr); // Initialize the ROS2
    // Create the publisher
    VideoPublisher video_publisher("camera0", 0, 100, 640, 640); // Topic name: camera, Device: 0, Time cycle: 100 ms, Height: 640, Width: 640
    video_publisher.createPublisher();
    video_publisher.publishVideo();
    // Spin the node
    rclcpp::spin(video_publisher.get_node_base_interface());
    // Exit the node
    rclcpp::shutdown();
    return 0;
}
