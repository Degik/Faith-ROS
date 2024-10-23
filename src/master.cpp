#include <iostream>
#include <FileCollector.hpp>
#include <VideoCollector.hpp>

int main(void){
    // Take the configuration from config.ini
    Collector collector("config.ini");
    // Save the camera ids
    vector_str camera_ids = collector.getCamerasIds();
    // Save the camera topics
    vector_str camera_topics = collector.getCamerasTopic();
    // Take the model type
    std::string model_name = collector.getModel();
    //
    rclcpp::init(0, nullptr);
    // Create the VideoCollector
    VideoCollector video_collector(camera_topics);
    // Create the subscriptions
    video_collector.createSubscription();
    // Spin the node
    rclcpp::spin(video_collector.get_node_base_interface());
    // Exit the node
    rclcpp::shutdown();
    return 0;
}