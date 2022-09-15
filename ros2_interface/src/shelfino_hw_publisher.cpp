#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "hardwareparameters.h"
#include "hardwareglobalinterface.h"
#include "json.hpp"

using namespace std::chrono_literals;
using namespace nlohmann;

//const auto R_ID_DEFAULT = "localhost";
const auto R_ID_DEFAULT = 2;
std::unique_ptr<HardwareParameters> hp;



void prova(){
  hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
  ZMQCommon::Subscriber sub;
  sub.register_callback([&](const char *topic, const char *buf, size_t size, void *data){
        nlohmann::json j;

        try {
          j = nlohmann::json::parse(std::string(buf, size));
          
          std::cout << j.dump() << std::endl;
        }
        catch(std::exception &e){
          //std::cerr << "error parsing: " << e.what() << std::endl;
        }
      });
  sub.start(hp->realSenseOdom, "ODOM");
  while(1){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

class ShelfinoHWPublisher : public rclcpp::Node
{
  public:
    ShelfinoHWPublisher()
    : Node("shelfino_hw_publisher")
    {
      lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      t265_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("t265", 10);
      lidar_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::lidar_callback, this));
      t265_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::t265_callback, this));
    }

  private:
    void lidar_callback()
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
      for(int i=0;i<lidarData.datum.size();i++){
        data.push_back(lidarData.datum.at(i).distance);
      }

      msg.ranges = data;

      lidar_publisher_->publish(msg);
    }
    void t265_callback()
    {
      RobotStatus::OdometryData odomData; 
      HardwareGlobalInterface::getInstance().getRealSenseOdomData(odomData);

      nav_msgs::msg::Odometry msg;
      msg.header.stamp = rclcpp::Node::now();
      msg.header.frame_id = "odom";
      msg.child_frame_id = "base_link";

      msg.pose.pose.position.x = odomData.pos_x; 
      msg.pose.pose.position.y = odomData.pos_y;
      msg.pose.pose.position.z = odomData.pos_z;
      msg.pose.pose.orientation.x = odomData.orient_x;
      msg.pose.pose.orientation.y = odomData.orient_y;
      msg.pose.pose.orientation.z = odomData.orient_z;
      msg.pose.pose.orientation.w = odomData.orient_w;

      msg.twist.twist.linear.x = odomData.twist_lin_x;
      msg.twist.twist.linear.y = odomData.twist_lin_y;
      msg.twist.twist.linear.z = odomData.twist_lin_z;
      msg.twist.twist.angular.x = odomData.twist_ang_x;
      msg.twist.twist.angular.y = odomData.twist_ang_y;
      msg.twist.twist.angular.z = odomData.twist_ang_z;

      for (int i=0; i<odomData.pose_cov.size(); i++) {
        msg.pose.covariance[i] = odomData.pose_cov.at(i);
        msg.twist.covariance[i] = odomData.twist_cov.at(i);
      }

      t265_publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr t265_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr t265_publisher_;
};

int main(int argc, char * argv[])
{
  hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
  HardwareGlobalInterface::initialize(hp.get());

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfinoHWPublisher>());

  rclcpp::shutdown();
  return 0;
}

