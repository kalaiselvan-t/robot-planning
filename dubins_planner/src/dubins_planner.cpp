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
using namespace std;

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

		// RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path");
		// std::cout << "Test var: " << pidx << std::endl;

		// publisher_->publish(path_msg);
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
	// Initialization

	init.x = X0;
	init.y = Y0;
	init.th = Th0;

	final.x = Xf;
	final.y = Yf;
	final.th = Thf;

	best_path.push_back(init);
	best_path.push_back(final);

	std::cout << best_path[0].y << std::endl;
	std::cout << best_path[1].y << std::endl;

	double best_path_len = std::numeric_limits<double>::max();

	if 
	(no_waypts > 2)
	{
		dubins_shortest_path(init.x, init.y, init.th, final.x, final.y, final.th, Kmax, pidx, dubin_curve);

		if
		(pidx > 0)
		{
			// int arc1_pts[no_of_samples][2];
			// int arc2_pts[no_of_samples][2];
			// int arc3_pts[no_of_samples][2];

			vector<vector<int>> arc1_pts(no_of_samples);
			vector<vector<int>> arc2_pts(no_of_samples);
			vector<vector<int>> arc2_pts(no_of_samples);

			dubinsarc_out arc1;
			dubinsarc_out arc2;
			dubinsarc_out arc3;

			arc1.x0 = init.x;
			arc1.y0 = init.y;

			plot_dubins(&dubin_curve, arc1_pts, arc2_pts, arc3_pts);
		}

	// std::cout << arc1_pts[0][0]

	}

	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<DubinsPathPublisher>());
  	rclcpp::shutdown();
  	return 0;

}

// double x1, y1, Kmax, step, angle_step;
		// point initial, final;
		// int no_of_wpts = 3;

		// point best_path[no_of_wpts-1];

		// initial.x = 2.0;
		// initial.y = 0.0;
		// initial.th = -2.0 / 3.0 * M_PI;

		// x1 = 5.0; 
		// y1 = 3.0;

		// final.x = 8.0;
		// final.y = 3.0;
		// final.th = -M_PI / 2.0;

		// Kmax = 3.0;

		// no_of_wpts = 3;
		// step = 6.0;
		// angle_step = M_PI / step;

		// best_path[0] = initial;
		// best_path[no_of_wpts-1] = final;




		// dubinsarc_out arc1;
		// arc1.x0 = 1.0;
		// arc1.y0 = 1.0;
		// arc1.th0 = 0.5;
		// arc1.xf = 2.0;
		// arc1.yf = 2.0;
		// arc1.thf = 0.8;
		// arc1.k = 2.0;
		// arc1.l = 2.0;

		// dubinsarc_out arc2;
		// arc2.x0 = 2.0;
		// arc2.y0 = 2.0;
		// arc2.th0 = 0.0;
		// arc2.xf = 5.0;
		// arc2.yf = 5.0;
		// arc2.thf = 0.0;
		// arc2.k = 2.0;
		// arc2.l = 2.0;

		// dubinsarc_out arc3;
		// arc3.x0 = 5.0;
		// arc3.y0 = 5.0;
		// arc3.th0 = 0.5;
		// arc3.xf = 6.0;
		// arc3.yf = 6.0;
		// arc3.thf = 0.8;
		// arc3.k = 2.0;
		// arc3.l = 2.0;

		// dubinscurve_out curve;

		// point initial;

		// curve.a1 = arc1;
		// curve.a2 = arc2;
		// curve.a3 = arc3;
		// curve.L = 5.0;

		// int c1[101][2];
		// int c2[101][2];
		// int c3[101][2];

		// // plotarc(&arc1, c1);
		// plot_dubins(&curve, true, c1, c2, c3);

		// std::cout << c1[10][0] << ", " << c1[10][1] << std::endl;
		// std::cout << c2[10][0] << ", " << c2[10][1] << std::endl;
		// std::cout << c3[10][0] << ", " << c3[10][1]<< std::endl;