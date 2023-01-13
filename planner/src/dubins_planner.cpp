#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "obstacles_msgs/msg/waypoints_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "dubins_planner/dubins_trajectory.h"
#include "geometry_msgs/msg/point32.hpp"


// typedef geometry_msgs::msg::Point32 Point;

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

//Initialize data structures

vector<vector<float>> arc1_pts;
vector<vector<float>> arc2_pts;
vector<vector<float>> arc3_pts;
std::vector<geometry_msgs::msg::Point32> w_pts;

// class DubinsPathPublisher: public rclcpp::Node
// {

// public:
	
// 	DubinsPathPublisher():Node("dubins_path_node"),count_(0)
// 	{
// 		publisher_ = this->create_publisher<nav_msgs::msg::Path>("dubins_path",10);
// 		sub_ = this->create_subscription<obstacles_msgs::msg::WaypointsMsg>("waypoints",1,std::bind(&DubinsPathPublisher::sub_callback, this, _1));
// 		timer_ = this->create_wall_timer(500ms, std::bind(&DubinsPathPublisher::timer_callback, this));
// 	}

// private:

	// void timer_callback()
	// {
	// 	nav_msgs::msg::Path path_msg;
	// 	tf2::Quaternion q; 

	// 	path_msg.header.stamp = this->get_clock()->now();
	// 	path_msg.header.frame_id = "map";
		
	// 	for 
	// 	(size_t i = 0; i < trajectory_arcs.size(); i++)
	// 	{
	// 		plotarc(&trajectory_arcs[i], arc1_pts);
	// 		geometry_msgs::msg::PoseStamped temp;
	// 		temp.header.stamp = this->get_clock()->now();
	// 		temp.header.frame_id = ' ';

	// 		for 
	// 		(int j = 0; j < no_of_samples; j++)
	// 		{
	// 			temp.pose.position.x = round_up((arc1_pts[j][0]),4);
	// 			temp.pose.position.y = round_up((arc1_pts[j][1]),4);
	// 			// cout << "temp; " << temp.pose.position.x << ", " << temp.pose.position.x << endl;  
	// 			temp.pose.position.z = 0;

	// 			q.setRPY(0, 0, arc1_pts[j][2]);

	// 			temp.pose.orientation.x = q.x();
	// 			temp.pose.orientation.y = q.y();
	// 			temp.pose.orientation.z = q.z();
	// 			temp.pose.orientation.w = q.w();
	// 			path_msg.poses.push_back(temp);
	// 		}
	// 		// cout << "=================================\n";
	// 		arc1_pts.clear();
	// 	}

	// 	RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path");

	// 	publisher_->publish(path_msg);
	// }

	// void sub_callback
	// (const obstacles_msgs::msg::WaypointsMsg::SharedPtr msg) const
	// {
	// 	// std::cout << "subscribed\n";
	// 	// RCLCPP_INFO(this->get_logger(), "subscribing");
	// 	cout << "way points x: " << msg->waypoints[0].x << endl;
	// }
	// void sub_callback(const obstacles_msgs::msg::WaypointsMsg msg) const
    // {
    // //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
	// 	std::cout << "subscribed\n";
    // }

	// void sub_callback
	// (const obstacles_msgs::msg::WaypointsMsg::SharedPtr msg) const
	// {
	// 	// for (size_t i = 0; i < msg.waypoints.size(); i++)
	// 	// {
	// 	// 	std::cout << "x: " << msg.waypoints[i].x << ", y: " << msg.waypoints[i].y << ", z: " << std::endl;
	// 	// }
	// 	// cout << "subscribed\n";
	// }

// 	rclcpp::TimerBase::SharedPtr timer_;
// 	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
// 	rclcpp::Subscription<obstacles_msgs::msg::WaypointsMsg>::SharedPtr sub_;
// 	size_t count_;

// };

// void wpts
// ()
// {
// 	for (size_t i = 0; i < waypoints.size(); i++)
// 	{
// 		point p;
// 		p.x = waypoints[i].x;
// 		p.y = waypoints[i].y;
// 		// p.z = waypoints[i].z;
// 		// w_pts.push_back(p);
// 	}
	
// }

void multipoints(vector<Pt> waypoints)
{
	// cout << "waypoints: " << waypoints[0].y << endl;
	// Create a list of intermediate paths
	// wpts();
	init.x = X0;
	init.y = Y0;
	init.th = Th0;

	final.x = Xf;
	final.y = Yf;
	final.th = Thf;

	best_path.clear();

	// best_path.push_back(init);
	// best_path.push_back(final);
	// for
	// (int i = 1; i <= no_waypts; i++)
	// {
	// 	// cout << i << endl;
	// 	if
	// 	(i == 1)
	// 	{
	// 		init.x = X0;
	// 		init.y = Y0;
	// 		init.th = Th0;
	// 		best_path.push_back(init);
	// 	}
	// 	else if
	// 	(i == no_waypts)
	// 	{
	// 		final.x = Xf;
	// 		final.y = Yf;
	// 		final.th = Thf;
	// 		best_path.push_back(final);
	// 	}
	// 	else
	// 	{
	// 		point temp;
	// 		temp.x = 3.0;
	// 		temp.y = 3.0;
	// 		temp.th = 0.0;
	// 		best_path.push_back(temp);
	// 	}
	// }

	// cout << "best Path size bfr: " << best_path.size() << endl;
	// best_path.erase(best_path.begin()+1);

	// best_path.insert(best_path.begin()+1,waypoints.begin(),waypoints.end());
	// best_path.insert(best_path.begin(),waypoints.begin(),waypoints.end());
	best_path = waypoints;
	// best_path.insert(best_path.begin()+1,w_pts.begin(),w_pts.end());


	// cout << "best Path size aft: " << best_path.size() << endl;

	// if
	// (true)
	// {
	// 	std::cout << "Main loop: " << std::endl;
	// 	std::cout << best_path[0].x << ", " << best_path[0].y << ", " << best_path[0].th << std::endl;
	// 	std::cout << best_path[1].x << ", " << best_path[1].y << ", " << best_path[1].th << std::endl;
	// 	std::cout << best_path[2].x << ", " << best_path[2].y << ", " << best_path[2].th << std::endl;
	// }
	/*
	=============================================================================================================
	*/
	float overall_length = 0.0;
	for
	(size_t i = best_path.size() - 2; i > 0; i--)
	{
		float best_path_length = std::numeric_limits<float>::max();
		int best_path_index;

		// cout << "Iteration: " << i << endl;

		for 
		(int k = 0; k < step; k++)
		{
			dubinscurve_out temp;
			float temp_ang = k * angle_step;
			dubins_shortest_path(best_path[i].x, best_path[i].y, temp_ang, best_path[i+1].x, best_path[i+1].y, best_path[i+1].th, Kmax, pidx, &temp);
			// cout << "index: " << k << "curve length: " << temp.L << endl;

			if ((temp.L + overall_length) < (best_path_length + overall_length))
			{
				best_path_length = temp.L;
				best_path_index = k;
				// cout << "best length found: " << best_path_length << endl;
			}
			// cout << "k: " << k << endl;
		}
		best_path[i].th = best_path_index * angle_step;
		overall_length = overall_length + best_path_length;
	}

	for (size_t i = 0; i < best_path.size(); i++)
	{
		cout << "x: " << best_path[i].x << ", y: " << best_path[i].y << ", th: " << best_path[i].th << endl;
		cout << "--------------------------\n";
	}
	
	/*
	=============================================================================================================
	*/
	// if
	// (true)
	// {
	// 	std::cout << "Seconary loop: " << std::endl;
	// 	std::cout << best_path[0].x << ", " << best_path[0].y << ", " << best_path[0].th << std::endl;
	// 	std::cout << best_path[1].x << ", " << best_path[1].y << ", " << best_path[1].th << std::endl;
	// 	std::cout << best_path[2].x << ", " << best_path[2].y << ", " << best_path[2].th << std::endl;
	// }
	/*
	=============================================================================================================
	*/
	for 
	(size_t i = 0; i < best_path.size()-1; i++)
	{
		dubinscurve_out temp;
		dubins_shortest_path(best_path[i].x, best_path[i].y, best_path[i].th, best_path[i+1].x, best_path[i+1].y, best_path[i+1].th, Kmax, pidx, &temp);
		
		// cout << "a1: " << temp.a1.x0 << ", " << temp.a1.y0 << ", " << temp.a1.th0 << endl;
		// cout << "a1: " << temp.a1.xf << ", " << temp.a1.yf << ", " << temp.a1.thf << endl;
		// cout << "a1.l: " << temp.a1.l << " a1.k: " << temp.a1.k << endl << endl;
		// cout << "a2: " << temp.a2.x0 << ", " << temp.a2.y0 << ", " << temp.a2.th0 << endl;
		// cout << "a2: " << temp.a2.xf << ", " << temp.a2.yf << ", " << temp.a2.thf << endl;
		// cout << "a2.l: " << temp.a2.l << " a2.k: " << temp.a2.yf << endl << endl;
		// cout << "a3: " << temp.a3.x0 << ", " << temp.a3.y0 << ", " << temp.a3.th0 << endl;
		// cout << "a3: " << temp.a3.xf << ", " << temp.a3.yf << ", " << temp.a3.thf << endl;
		// cout << "a3.l: " << temp.a3.l << " a3.k " << temp.a3.k << endl << endl;

		trajectory_arcs.push_back(temp.a1);
		// cout << "a1.l: " << temp.a1.l << endl; 
		trajectory_arcs.push_back(temp.a2);
		// cout << "a2.l: " << temp.a2.l << endl;
		trajectory_arcs.push_back(temp.a3);
		// cout << "a3.l: " << temp.a3.l << endl;
	}
	/*
	=============================================================================================================
	*/

	// cout << "Traj size: " << trajectory_arcs.size() << endl;

	// for 
	// (size_t i = 0; i < trajectory_arcs.size(); i++)
	// {
	// 	cout << "Trajector points ind: " << i << endl;
	// 	// cout << "a1: " << trajectory_points[i].a1.x0 << ", " << trajectory_points[i].a1.y0 << ", " << trajectory_points[i].a1.th0 << endl;
	// 	// cout << "a1: " << trajectory_points[i].a1.xf << ", " << trajectory_points[i].a1.yf << ", " << trajectory_points[i].a1.thf << endl;
	// 	// cout << "a2: " << trajectory_points[i].a2.x0 << ", " << trajectory_points[i].a2.y0 << ", " << trajectory_points[i].a2.th0 << endl;
	// 	// cout << "a2: " << trajectory_points[i].a2.xf << ", " << trajectory_points[i].a2.yf << ", " << trajectory_points[i].a2.thf << endl;
	// 	// cout << "a3: " << trajectory_points[i].a3.x0 << ", " << trajectory_points[i].a3.y0 << ", " << trajectory_points[i].a3.th0 << endl;
	// 	// cout << "a3: " << trajectory_points[i].a3.xf << ", " << trajectory_points[i].a3.yf << ", " << trajectory_points[i].a3.thf << endl << endl;
	// 	cout << "a: " << trajectory_arcs[i].x0 << ", " << trajectory_arcs[i].y0 << ", " << trajectory_arcs[i].th0 << endl;
	// 	cout << "a: " << trajectory_arcs[i].xf << ", " << trajectory_arcs[i].yf << ", " << trajectory_arcs[i].thf << endl << endl;
	// }
	

	// dubins_shortest_path(init.x, init.y, init.th, final.x, final.y, final.th, Kmax, pidx, &dubin_curve);

}
