#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "dubins_planner/dubins_trajectory.h"

using namespace std::chrono_literals;

class DubinsPathPublisher: public rclcpp::Node
{

public:
	
	DubinsPathPublisher():Node("dubins_path_node"),count_(0)
	{
		publisher_ = this->create_publisher<nav_msgs::msg::Path>("dubins_path",10);
		timer_ = this->create_wall_timer(500ms, std::bind(&DubinsPathPublisher::timer_callback, this));
	}

private:

	void timer_callback()
	{
		nav_msgs::msg::Path path_msg;
		geometry_msgs::msg::PoseStamped temp; 

		path_msg.header.stamp = this->get_clock()->now();
		path_msg.header.frame_id = "map";

		temp.header.stamp = this->get_clock()->now();
		temp.header.frame_id = ' ';
		
		temp.pose.position.x = 0;
		temp.pose.position.y = 0;
		temp.pose.position.z = 0;

		temp.pose.orientation.x = 0;
		temp.pose.orientation.y = 0;
		temp.pose.orientation.z = 0;
		temp.pose.orientation.w = 1;
		path_msg.poses.push_back(temp);

		temp.pose.position.x = 4;
		temp.pose.position.y = 6;
		temp.pose.position.z = 0;

		temp.pose.orientation.x = 0;
		temp.pose.orientation.y = 0;
		temp.pose.orientation.z = 0;
		temp.pose.orientation.w = 1;
		path_msg.poses.push_back(temp);

		RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path");
		std::cout << "Test var: " << pidx << std::endl;

		publisher_->publish(path_msg);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
	size_t count_;

};

// 	// dubinscurve_out dubin_curve;

// 	// dubins_shortest_path(X0, Y0, Th0, Xf, Yf, Thf, Kmax, pidx, &dubin_curve);

// 	// std::cout << "Pidx: " << pidx << std::endl;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DubinsPathPublisher>());
  rclcpp::shutdown();
  return 0;
}