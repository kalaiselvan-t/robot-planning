#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"




class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("follow_path")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("obstacles", 10);

        geometry_msgs::msg::Polygon msg;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        geometry_msgs::msg::Point32 point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 0;
        point.y = 1;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 1;
        point.y = 1;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 1;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        msg.points = points_temp;
        while(1){
          publisher_->publish(msg);
          usleep(1000000);
        }
    }

  
  private:
    
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}