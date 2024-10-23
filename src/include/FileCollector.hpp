#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <string>
#include <vector>

class Collector {
public:
    Collector(const std::string& filename);
    std::vector<std::string> getCamerasTopic() const;
    std::vector<std::string> getCamerasIds() const;
    std::string getModel() const;

private:
    boost::property_tree::ptree config_;
};