#include "FileCollector.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// This class is used to read the configuration file
Collector::Collector(const std::string& filename) {
    boost::property_tree::ini_parser::read_ini(filename, config_);
}

// Get the cameras topic
std::vector<std::string> Collector::getCamerasTopic() const {
    std::vector<std::string> cameras_topic;
    for (const auto& item : config_.get_child("cameras_topic")) {
        cameras_topic.push_back(item.second.data());
    }
    return cameras_topic;
}
// Get canera ids
std::vector<std::string> Collector::getCamerasIds() const {
    std::vector<std::string> cameras_ids;
    for (const auto& item : config_.get_child("cameras_ids")) {
        cameras_ids.push_back(item.second.data());
    }
    return cameras_ids;
}
// Get the model
std::string Collector::getModel() const {
    return config_.get<std::string>("model_name.model");
}
