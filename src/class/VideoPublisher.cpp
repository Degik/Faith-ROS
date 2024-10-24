#include "VideoPublisher.hpp"

// Constructor
VideoPublisher::VideoPublisher(std::string topic_name, int device, int time_cycle, int height = 1280, int width = 720) : Node("camera_node"){
    // Initialize the attributes
    this->topic_name = topic_name;
    this->device = device;
    this->time_cycle = time_cycle;
    // Initialize the status
    this->working = false;
    //this->qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
}

void VideoPublisher::createPublisher(){
    // Create the publisher
    std::cout << "Info: The QoS size is not defined" << std::endl;
    std::cout << "The QoS profile will be automatic" << std::endl;
    this->autoQoS();
    std::cout << "The QoS profile was set" << std::endl;
    this->publisher = this->create_publisher<sensor_msgs::msg::Image>(this->topic_name, this->qos);
    std::cout << "The publisher was created" << std::endl;
}

void VideoPublisher::publishVideo(){
    // Open the video
    this->capture.open(this->device);
    // Check if the video is open
    if(!this->capture.isOpened()){
        std::cerr << "Error: The video can't be opened" << std::endl;
        return;
    }
    // Set the width and height
    this->capture.set(cv::CAP_PROP_FRAME_WIDTH, this->width);
    this->capture.set(cv::CAP_PROP_FRAME_HEIGHT, this->height);
    // Set the status
    this->working = true;
    // Create the timer
    this->timer = this->create_wall_timer(std::chrono::milliseconds(this->time_cycle), std::bind(&VideoPublisher::sendImage, this));
}

void VideoPublisher::sendImage(){
    //
    cv::Mat frame;
    this->capture >> frame;

    if(!frame.empty()){
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        this->publisher->publish(*msg);
    }
}

void VideoPublisher::autoQoS(){
    // https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
    // Setting the QoS profile
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    //qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    this->qos = qos;

    if(this->working){
        std::cerr << "The video is working, you can't modify the QoS profile" << std::endl;
        return;
    }
}

VideoPublisher::~VideoPublisher(){
    // Release the capture
    capture.release();
}

