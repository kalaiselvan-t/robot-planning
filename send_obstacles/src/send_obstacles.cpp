#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"


class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("follow_path")
    {
        publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", 10);

        std_msgs::msg::Header hh;

        hh.stamp = this->get_clock()->now();
        hh.frame_id = "map";

        obstacles_msgs::msg::ObstacleArrayMsg msg;
        obstacles_msgs::msg::ObstacleMsg obs;
        std::vector<obstacles_msgs::msg::ObstacleMsg> obs_temp;
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;

        geometry_msgs::msg::PolygonStamped pol1;
        geometry_msgs::msg::PolygonStamped pol2;

        pol1.header = hh;
        pol2.header = hh;

        // First square obstacle
        {
          std::vector<geometry_msgs::msg::Point32> points_temp;
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
          pol.points = points_temp;
          obs.polygon = pol;
          obs_temp.push_back(obs);
          pol1.polygon = pol;
        }

        // First square obstacle
        {
          std::vector<geometry_msgs::msg::Point32> points_temp;
          point.x = 5;
          point.y = 5;
          point.z = 0;
          points_temp.push_back(point);
          point.x = 5;
          point.y = 6;
          point.z = 0;
          points_temp.push_back(point);
          point.x = 6;
          point.y = 6;
          point.z = 0;
          points_temp.push_back(point);
          point.x = 6;
          point.y = 5;
          point.z = 0;
          points_temp.push_back(point);
          pol.points = points_temp;
          obs.polygon = pol;
          obs_temp.push_back(obs);
          pol2.polygon = pol;
        }

        msg.obstacles = obs_temp;

        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub1 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("obs1", 10);
        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub2 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("obs2", 10);

        while(rclcpp::ok()){
          publisher_->publish(msg);
          pub1->publish(pol1);
          pub2->publish(pol2);
          usleep(1000000);
        }
    }

  
  private:
    
    rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}