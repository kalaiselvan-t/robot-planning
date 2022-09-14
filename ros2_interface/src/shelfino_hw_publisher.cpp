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

class ShelfinoHWPublisher : public rclcpp::Node
{
  public:
    ShelfinoLidarPublisher()
    : Node("shelfino_hw_publisher")
    {
      lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      lidar_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::lidar_callback, this));
    }

  private:void lidar_callback()
    {
      RobotStatus::LidarData lidarData; 
      HardwareGlobalInterface::getInstance().getFrontLidarData(lidarData);

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = rclcpp::Node::now();
      msg.header.frame_id = "base_laser";
      msg.angle_increment = 0.00872664625;
      msg.angle_min = msg.angle_increment;
      msg.angle_max = 6.27445866092 + msg.angle_min;
      msg.time_increment = (1./10.) / lidarData.datum.size();
      msg.scan_time = 1./10.;
      msg.range_min = 0.05;
      msg.range_max = 10;

      std::vector<float> data;
      for(auto i : lidarData.datum){
        data.push_back(i.distance);
      }

      msg.ranges = data;

      lidar_publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
  HardwareGlobalInterface::initialize(hp.get());
  HardwareGlobalInterface::getInstance().robotOnVelControl();
  HardwareGlobalInterface::getInstance().robotOff();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfinoHWPublisher>());

  rclcpp::shutdown();
  return 0;
}