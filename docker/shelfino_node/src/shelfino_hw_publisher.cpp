#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      t265_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("t265", 10);
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      encoders_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      lidar_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::lidar_callback, this));
      t265_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::t265_callback, this));
      odom_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::odom_callback, this));
      encoders_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWPublisher::enc_callback, this));
      cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ShelfinoHWPublisher::handle_shelfino_cmd_vel, this, std::placeholders::_1));
    }

  private:
    void lidar_callback()
    {
      RobotStatus::LidarData lidarData; 
      HardwareGlobalInterface::getInstance().getFrontLidarData(lidarData);

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = this->get_clock()->now();
      
      msg.header.frame_id = "base_laser";
      msg.angle_increment = 0.00872664625;
      msg.angle_min = msg.angle_increment;
      msg.angle_max = 6.27445866092 + msg.angle_min;
      //msg.time_increment = (1./10.) / lidarData.datum.size();
      //msg.scan_time = 1./10.;
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
      msg.header.stamp = this->get_clock()->now();
      
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

    void odom_callback()
    {
      RobotStatus::OdometryData odomData; 
      HardwareGlobalInterface::getInstance().getOdomData(odomData);

      nav_msgs::msg::Odometry msg;
      msg.header.stamp = this->get_clock()->now();

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

      odom_publisher_->publish(msg);
      handle_shelfino_pose(msg);
    }

    void enc_callback()
    {
      RobotStatus::HardwareData hwData; 
      HardwareGlobalInterface::getInstance().getHardwareData(hwData);

      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->get_clock()->now();

      msg.header.frame_id = "";

      msg.name.push_back("wheel_left_joint");
      msg.position.push_back(hwData.leftWheel.ticks);
      msg.velocity.push_back(hwData.leftWheel.omega);
      msg.name.push_back("wheel_right_joint");
      msg.position.push_back(hwData.rightWheel.ticks);
      msg.velocity.push_back(hwData.rightWheel.omega);

      encoders_publisher_->publish(msg);
    }

    void handle_shelfino_cmd_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
      double v = 0., omega = 0.;
      v = msg->linear.x;
      omega = msg->angular.z;
      if(v != 0 || omega != 0){
        if(!robot_state){
          robot_state = true;
          HardwareGlobalInterface::getInstance().robotOnVelControl();
        }
      } else {
        robot_state = false;
        HardwareGlobalInterface::getInstance().robotOff();
        return;
      }

      HardwareGlobalInterface::getInstance().vehicleMove(v,omega);

      return;
    }

    void handle_shelfino_pose(nav_msgs::msg::Odometry msg)
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = msg.header.stamp;

      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";

      t.transform.translation.x = msg.pose.pose.position.x;
      t.transform.translation.y = msg.pose.pose.position.y;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = msg.pose.pose.orientation.x;
      t.transform.rotation.y = msg.pose.pose.orientation.y;
      t.transform.rotation.z = msg.pose.pose.orientation.z;
      t.transform.rotation.w = msg.pose.pose.orientation.w;

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    bool robot_state = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr t265_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr encoders_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr t265_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoders_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
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