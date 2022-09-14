#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "json.hpp"

#include "hardwareparameters.h"
#include "hardwareglobalinterface.h"

using namespace std::chrono_literals;
using namespace nlohmann;

//const auto R_ID_DEFAULT = "localhost";
const auto R_ID_DEFAULT = 2;
std::unique_ptr<HardwareParameters> hp;

class ShelfinoLidarPublisher : public rclcpp::Node
{
  public:
    ShelfinoLidarPublisher()
    : Node("shelfino_lidar_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      subFrontLidar.register_callback([&](const char *topic, const char *buf, size_t size, void *data){
        nlohmann::json j;

        try {
          j = nlohmann::json::parse(std::string(buf, size));
          int size = j.at("size");
          std::vector<float> data = j.at("data");

          sensor_msgs::msg::LaserScan msg;
          //msg.header.seq = id++;
          msg.header.stamp = rclcpp::Node::now();
          msg.header.frame_id = "base_laser";
          msg.angle_increment = 0.00872664625;
          msg.angle_min = msg.angle_increment;
          msg.angle_max = 6.27445866092 + msg.angle_min;
          msg.time_increment = (1./10.) / size;
          msg.scan_time = 1./10.;
          msg.range_min = 0.05;
          msg.range_max = 10;
          msg.ranges = data;
          //msg.intensities = std::vector<float>(size, 0.);

          publisher_->publish(msg);
        }
        catch(std::exception &e){
          //std::cerr << "error parsing: " << e.what() << std::endl;
        }
      });
      subFrontLidar.start(hp->frontLidarPublisher, "LIDAR");
    }

  private:
    ZMQCommon::Subscriber subFrontLidar;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
  HardwareGlobalInterface::initialize(hp.get());
  HardwareGlobalInterface::getInstance().robotOnVelControl();

  RobotStatus::LidarData lidarData; 
  HardwareGlobalInterface::getInstance().getFrontLidarData(lidarData);
  std::cout << lidarData.lidarTimer << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  HardwareGlobalInterface::getInstance().getFrontLidarData(lidarData);
  std::cout << lidarData.lidarTimer << std::endl;

  HardwareGlobalInterface::getInstance().robotOff();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfinoLidarPublisher>());

  rclcpp::shutdown();
  return 0;
}