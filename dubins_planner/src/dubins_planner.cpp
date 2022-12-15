#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "dubins_planner/dubins_trajectory.h"

using namespace std::chrono_literals;
using namespace std;

//Initialize data structures

vector<vector<double>> arc1_pts;
vector<vector<double>> arc2_pts;
vector<vector<double>> arc3_pts;

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
		tf2::Quaternion q; 

		path_msg.header.stamp = this->get_clock()->now();
		path_msg.header.frame_id = "map";

		temp.header.stamp = this->get_clock()->now();
		temp.header.frame_id = ' ';

		plotarc(&dubin_curve.a1, arc1_pts);

		for 
		(int i = 0; i <= no_of_samples-1; ++i)
		{
			for
			(int j = 0; j < 2; ++j)
			{
				temp.pose.position.x = round_up((arc1_pts[i][0]),2);
				temp.pose.position.y = round_up((arc1_pts[i][1]),2);
				temp.pose.position.z = 0;

				if
				(DEBUG)
				{
					temp.pose.position.x = 1;
					temp.pose.position.y = 1;
				}
				

            	q.setRPY(0, 0, arc1_pts[i][2]);

				temp.pose.orientation.x = q.x();
				temp.pose.orientation.y = q.y();
				temp.pose.orientation.z = q.z();
				temp.pose.orientation.w = q.w();
				path_msg.poses.push_back(temp);

				if
				(DEBUG)
				{// temp.pose.position.x = 3;
					temp.pose.position.y = 4;
					temp.pose.position.z = 0;
	
					temp.pose.orientation.x = 0;
					temp.pose.orientation.y = 0;
					temp.pose.orientation.z = 0;
					temp.pose.orientation.w = 1;
					path_msg.poses.push_back(temp);
	
					temp.pose.position.x = 9;
					temp.pose.position.y = 3;
					temp.pose.position.z = 0;
	
					temp.pose.orientation.x = 0;
					temp.pose.orientation.y = 0;
					temp.pose.orientation.z = 0;
					temp.pose.orientation.w = 1;
					path_msg.poses.push_back(temp);
				}
			}
		}

		plotarc(&dubin_curve.a2, arc2_pts);

		for 
		(int i = 1; i <= no_of_samples; ++i)
		{
			for
			(int j = 0; j < 2; ++j)
			{
				temp.pose.position.x = round_up((arc2_pts[i][0]),2);
				temp.pose.position.y = round_up((arc2_pts[i][1]),2);
				temp.pose.position.z = 0;

				q.setRPY(0, 0, arc2_pts[i][2]);

				temp.pose.orientation.x = q.x();
				temp.pose.orientation.y = q.y();
				temp.pose.orientation.z = q.z();
				temp.pose.orientation.w = q.w();
				path_msg.poses.push_back(temp);
			}
		}

		plotarc(&dubin_curve.a3, arc3_pts);

		for 
		(int i = 1; i <= no_of_samples; ++i)
		{
			for
			(int j = 0; j < 2; ++j)
			{
				temp.pose.position.x = round_up((arc3_pts[i][0]),2);
				temp.pose.position.y = round_up((arc3_pts[i][1]),2);
				// temp.pose.position.x = 1;
				// temp.pose.position.y = 1;
				temp.pose.position.z = 0;

				q.setRPY(0, 0, arc3_pts[i][2]);

				temp.pose.orientation.x = q.x();
				temp.pose.orientation.y = q.y();
				temp.pose.orientation.z = q.z();
				temp.pose.orientation.w = q.w();
				path_msg.poses.push_back(temp);
			}
		}

		RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path");

		publisher_->publish(path_msg);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
	size_t count_;

};

int main(int argc, char * argv[])
{
	// Initialization

	init.x = X0;
	init.y = Y0;
	init.th = Th0;

	final.x = Xf;
	final.y = Yf;
	final.th = Thf;

	std::cout << init.x << ", " << init.y << std::endl;
	std::cout << final.x << ", " << final.y << std::endl;

	for
	(int i = 0; i < no_waypts; i++)
	{
		if
		(i == 0)
		{
			auto pos = best_path.begin() + i;
			best_path.insert(pos,init);
		}
		else if
		(i == no_waypts+1)
		{
			auto pos = best_path.begin()+i;
			best_path.insert(pos,final);
		}
		else
		{
			point temp;
			temp.x, temp.y, temp.th = 0.0;
			auto pos = best_path.begin()+i;
			best_path.insert(pos,temp);
		}
	}

	if
	(true)
	{
		std::cout << "Main loop: " << std::endl;
		std::cout << best_path.size() << std::endl;
		std::cout << best_path[0].x << ", " << best_path[0].y << std::endl;
		std::cout << best_path[1].x << ", " << best_path[1].y << std::endl;
		std::cout << best_path[2].x << ", " << best_path[2].y << std::endl;
		// std::cout << best_path[1].y << std::endl;
		// std::cout << best_path[1].y << std::endl;
	}

	for
	(int i = no_waypts - 1; i > 0; i--)
	{

	}

	dubins_shortest_path(init.x, init.y, init.th, final.x, final.y, final.th, Kmax, pidx, &dubin_curve);

	// if
	// (DEBUG)
	// {
	// 	plotarc(&dubin_curve.a1, arc1_pts);
	// 	plotarc(&dubin_curve.a2, arc2_pts);
	// 	plotarc(&dubin_curve.a3, arc3_pts);

	// 	cout << "a1: " << dubin_curve.a1.x0 << ", " << dubin_curve.a1.y0 << endl;
	// 	cout << "a1.start: " << arc1_pts[0][0] << ", " << arc1_pts[0][1] << endl;
	// 	cout << "a1.final: " << arc1_pts[no_of_samples][0] << ", " << arc1_pts[no_of_samples][1] << endl;
	// 	cout << "a1.l: " << dubin_curve.a1.l << endl;
	// 	cout << "a2: " << dubin_curve.a2.x0 << ", " << dubin_curve.a2.y0 << endl;
	// 	cout << "a2.start: " << arc2_pts[0][0] << ", " << arc2_pts[0][1] << endl;
	// 	cout << "a2.final: " << arc2_pts[no_of_samples][0] << ", " << arc2_pts[no_of_samples][1] << endl;
	// 	cout << "a2.l: " << dubin_curve.a2.l << endl;
	// 	cout << "a3: " << dubin_curve.a3.x0 << ", " << dubin_curve.a3.y0 << endl;
	// 	cout << "a3.start: " << arc3_pts[0][0] << ", " << arc3_pts[0][1] << endl;
	// 	cout << "a3.final: " << arc3_pts[no_of_samples][0] << ", " << arc3_pts[no_of_samples][1] << endl;
	// 	cout << "a3.l: " << dubin_curve.a3.l << endl;
	// 	cout << "a1.y0: " << dubin_curve.a1.y0 << endl;
	// 	cout << "a2.x: " << dubin_curve.a2.x0 << endl;
	// 	cout << "a2.y: " << dubin_curve.a2.y0 << endl;
	// 	cout << "a3.x: " << dubin_curve.a3.x0 << endl;
	// 	cout << "a3.y: " << dubin_curve.a3.y0 << endl;

	// 	cout << "pidx: " << pidx << endl;
	// 	cout<< "L: " << dubin_curve.a1.l << endl;
	// }

	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<DubinsPathPublisher>());
  	rclcpp::shutdown();
  	return 0;

}
