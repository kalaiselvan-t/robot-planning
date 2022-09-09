#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "Subscriber.hpp"

using namespace std::chrono_literals;

class ShelfinoLidarPublisher : public rclcpp::Node
{
  public:
    ShelfinoLidarPublisher()
    : Node("shelfino_lidar_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("scan", 10);
      //timer_ = this->create_wall_timer(
      //500ms, std::bind(&ShelfinoLidarPublisher::timer_callback, this));
      ZMQCommon::Subscriber subFrontLidar;
      subFrontLidar.register_callback(&ShelfinoLidarPublisher::zmq_callback);
      subFrontLidar.start(frontLidarPublisher, "LIDAR");
    }

  private:
    void zmq_callback(const char *topic, const char *buf, size_t size, void *data)
    {
      nlohmann::json j;

      try {
        j = nlohmann::json::parse(std::string(buf, size));
        int size = j.at("size");
        std::vector<float> data = j.at("data");

        sensor_msgs::LaserScan msg;
        msg.header.seq = id++;
        msg.header.stamp = ros::Time::now();
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

        publisher.publish(msg);

        auto message = std_msgs::msg::String();
        message.data = std::to_string(count_++);
        publisher_->publish(message);
      }
      catch(std::exception &e){
        //std::cerr << "error parsing: " << e.what() << std::endl;
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int i = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfinoLidarPublisher>());
  rclcpp::shutdown();
  return 0;
}