#include <iostream>
#include "dubins-trajectory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

// int main(){
	
// 										//Problem Data
// 	double X0 = 5.0;
// 	double Y0 = 5.0;
// 	double Xf = 4.0;
// 	double Yf = 0.0;
// 	double Th0 = -2.0/3.0 * M_PI;
// 	double Thf = M_PI / 3.0;
// 	double Kmax = 3.0;

// 	int pidx = 0;
// 	dubinscurve_out dubin_curve;

// 	dubins_shortest_path(X0, Y0, Th0, Xf, Yf, Thf, Kmax, pidx, &dubin_curve);

// 	std::cout << "Pidx: " << pidx << std::endl;
// }

class TrajectoryPublisher: public rclcpp::Node
{
	public:

		TrajectoryPublisher()
		: Node("dubins-trajectory-publisher"), count_(0)
		{
			publisher_ = this->create_publisher<std::msgs::msg::String>("trajectory", 10);
			timer_ = this->create_wall_timer(500ms, std::bind(&TrajectoryPublisher::timer_callback, this));
		}
}

int int main
(int argc, char const *argv[])
{
	
	return 0;
}
