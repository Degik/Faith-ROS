#include <VideoCollector.hpp>

VideoCollector::VideoCollector(vector_str camera_topics) : Node("master_node"){
    // Take the camera topics
    this->camera_topics = camera_topics;
    // create the topic folder and the subtopics
    createFolders();
    createSubscription();
}

VideoCollector::~VideoCollector(){
    // Destroy the subscriptions
    for (int i = 0; i < subscriptions.size(); i++){
        subscriptions[i].reset();
    }
    // free the memory
    subscriptions.clear();

    // Release the video writers
    for (auto& writer : video_writers) {
        if (writer.isOpened()) {
            writer.release();
        }
    }
    video_writers.clear();
}

void VideoCollector::createSubscription(){
    video_writers.resize(camera_topics.size());
    // Create the subscriptions
    for (int i = 0; i < camera_topics.size(); i++){
        auto callback = [this, i](msg_ptr msg) -> void {
            // take the elaborated frame
            takeFrame(i, msg);
        };
        std::cout << "inf_" + camera_topics[i] << std::endl;
        subscriptions.push_back(create_subscription<sensor_msgs::msg::Image>("inf_" + camera_topics[i], 10, callback));
    }
}

void VideoCollector::takeFrame(int index, msg_ptr msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
        RCLCPP_ERROR(get_logger(), "(cv_bridge) exception: %s", e.what());
        return;
    }
    // Check if the image is empty
    if (cv_ptr->image.empty()){
        std::cout << "The image is empty topics"+index << std::endl;
        RCLCPP_ERROR(get_logger(), "The image is empty");
        return;
    }
    //
    std::string folder = "Videos/" + this->camera_topics[index];
    std::string video = folder + "/" + this->camera_topics[index] + ".avi";
    // Only for debug
    //
    cv::imshow("camera_" + index, cv_ptr->image);
    cv::waitKey(1);
    //
    // Check if the folder exist
    if (!std::filesystem::exists(folder)){
        RCLCPP_ERROR(get_logger(), "The folder %s doesn't exist", folder.c_str());
        return;
    }
    // Check if exist an video with the same topic video
    // If not create one else add the frame at the end of the video
    // 
    // Check if the video writer is open
    std::cout << "I'm writing the video" << video << std::endl;
    if (!video_writers[index].isOpened()) {
        int fps = 10;
        cv::Size frame_size(cv_ptr->image.cols, cv_ptr->image.rows);
        auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

        video_writers[index].open(video, fourcc, fps, frame_size, true);
        // Check if the video writer is open
        if (!video_writers[index].isOpened()) {
            RCLCPP_ERROR(get_logger(), "Could not open the output video file for write: %s", video.c_str());
            return;
        }
    }
    video_writers[index].write(cv_ptr->image);
}

void VideoCollector::createFolders(){
    // Create root folder
    std::filesystem::create_directory("Videos");
    // Create the folders
    for (int i = 0; i < camera_topics.size(); i++){
        std::string folder = "Videos/" + camera_topics[i];
        std::filesystem::create_directory(folder);
    }
}