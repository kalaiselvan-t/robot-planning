#include "rclcpp/rclcpp.hpp"
#include "../include/controller/common.hpp"
#include "../include/controller/data.hpp"
#include "../include/controller/a_star.hpp"

using std::placeholders::_1;

class Controller: public rclcpp::Node
{
    public:
        Controller() : Node("controller_node")
        {
			const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
			
			border_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
				"map_borders", 10, std::bind(&Controller::get_border, this, _1));

	  		gate_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
				"gate_position", qos, std::bind(&Controller::get_gate_poses, this, _1));

	  		obs_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
				"inflated_obstacles", 1, std::bind(&Controller::get_obs, this, _1));

	  		robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
				"transform", 1, std::bind(&Controller::get_robot_pose, this, _1));

	  		pub1_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/path", 1);

			pub2_ = this->create_publisher<nav_msgs::msg::Path>("shelfino2/path", 1);

	  		timer_ = this->create_wall_timer(
      			1000ms, std::bind(&Controller::timer_callback, this));
			
			// gridmap.create();
			// gridmap.print();
			mapBuilt = false;
			pathSent = false;
        }
    
    private:
		rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr border_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_poses_sub_;
		rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obs_sub_;
		rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot_pose_sub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub2_;
		rclcpp::TimerBase::SharedPtr timer_;

		AStarPlanner planner;
		bool mapBuilt;
		bool pathSent;
		
		size_t count_;
		GridMap gridmap;

		vector<Dubins_arc> path1_arcs;
		vector<Dubins_arc> path2_arcs;
		vector<vector<float>> points1;
		vector<vector<float>> points2;
		// std::string target_frame_;
		
        void get_border(const geometry_msgs::msg::Polygon msg);
		void get_gate_poses(const geometry_msgs::msg::PoseArray msg);
		void get_obs(const obstacles_msgs::msg::ObstacleArrayMsg msg);
		void timer_callback();
		void remove_obs_in_grid();
		void plotarc(const Dubins_arc& arc, std::vector<std::vector<float>> &points);
		void path1_publisher(const vector<Dubins_arc>& dubinsPath);
		void path2_publisher();
		void circline(float s, float x0, float y0, float th0, float k, float &x, float &y, float &th);
		void get_robot_pose(const geometry_msgs::msg::TransformStamped msg);
};

void Controller::timer_callback()
{
	// Planner planner = Planner(rp,ep,gridmap);
	// this->path1_arcs = planner.plan();
	// path1_publisher();

	if (this->mapBuilt && !this->pathSent)
	{
		const auto& path = planner.FindPath(
			boost::geometry::model::d2::point_xy<float>(-100.0, -100.0),
			boost::geometry::model::d2::point_xy<float>(100.0, 100.0)
		);

		std::cout << "planner path length: " << path.size() << std::endl;

		// convert points to waypoints
		std::vector<Pose2d> waypoints;
		for (const auto& point : path | boost::adaptors::indexed(0))
		{
			Pose2d waypoint;
			// pose.x = follow_path[i].first.ros_pose.position.x;
			// pose.y = follow_path[i].first.ros_pose.position.y;
			waypoint.x = point.value().x();
			waypoint.y = point.value().y();

			tf2::Quaternion q(tf2::Quaternion::getIdentity());
			// switch (point.index())
			// {
			// 	case 0:
			// 	break;
			// 	case path.size()-1:
			// 	break;
			// }

			tf2::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			waypoint.th = yaw;

			waypoints.push_back(waypoint);
		}
		std::cout << "waypoints count " << path.size() << std::endl;

		Planner p;
		auto dubinsPath = p.multipoints(waypoints);
		std::cout << "dubinsPath length " << dubinsPath.size() << std::endl;
		path1_publisher(dubinsPath);

		this->pathSent = true;
	}

	// float 
	// cout << "timer\n";
	// if (!frame_flag)
	// {
	// 	get_robot_pose();
	// 	frame_flag = true;
	// }
	// cout << "robot x: " << robot_pose.position.x << ", y: " << robot_pose.position.y << endl;
	// for(int i = 0; i < no_of_robots; i++)
	// {
		// geometry_msgs::msg::Pose rp;
		// geometry_msgs::msg::Pose ep;
		// rp.position.x = 2.0;
		// rp.position.y = 2.0;
		// ep.position.x = 5.0;
		// ep.position.y = 7.0;
		// Planner planner = Planner(rp,ep,gridmap);
		// this->path1_arcs = planner.plan();
		// path1_publisher();
		// geometry_msgs::msg::Pose rp1;
		// geometry_msgs::msg::Pose ep1;
		// rp1.position.x = 4.0;
		// rp1.position.y = 2.0;
		// ep1.position.x = 5.0;
		// ep1.position.y = 7.0;
		// Planner planner1 = Planner(rp1,ep1,gridmap);
		// this->path2_arcs = planner1.plan();
		// path2_publisher();
	// }
}

void Controller::path1_publisher(const vector<Dubins_arc>& dubinsPath)
{
	nav_msgs::msg::Path path_msg;
	tf2::Quaternion q; 

	path_msg.header.stamp = this->get_clock()->now();
	path_msg.header.frame_id = "map";
	
	std::vector<std::vector<float>> points;
	for 
	(size_t i = 0; i < dubinsPath.size(); i++)
	{
		plotarc(dubinsPath[i], points);
		geometry_msgs::msg::PoseStamped temp;
		temp.header.stamp = path_msg.header.stamp;
		temp.header.frame_id = ' ';

		for 
		(int j = 0; j < no_of_samples; j++)
		{
			temp.pose.position.x = round_up((points[j][0]),4);
			temp.pose.position.y = round_up((points[j][1]),4);
			// cout << "temp; " << temp.pose.position.x << ", " << temp.pose.position.x << endl;  
			temp.pose.position.z = 0;

			q.setRPY(0, 0, points[j][2]);

			temp.pose.orientation.x = q.x();
			temp.pose.orientation.y = q.y();
			temp.pose.orientation.z = q.z();
			temp.pose.orientation.w = q.w();
			path_msg.poses.push_back(temp);
		}
		// cout << "=================================\n";
		points.clear();
	}

	// RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path");

	pub1_->publish(path_msg);
}

void Controller::path2_publisher()
{
	nav_msgs::msg::Path path_msg;
	tf2::Quaternion q; 

	path_msg.header.stamp = this->get_clock()->now();
	path_msg.header.frame_id = "map";
	
	std::vector<std::vector<float>> points;
	for 
	(size_t i = 0; i < this->path2_arcs.size(); i++)
	{
		plotarc(this->path2_arcs[i], points);
		geometry_msgs::msg::PoseStamped temp;
		temp.header.stamp = this->get_clock()->now();
		temp.header.frame_id = ' ';

		for 
		(int j = 0; j < no_of_samples; j++)
		{
			temp.pose.position.x = round_up((points[j][0]),4);
			temp.pose.position.y = round_up((points[j][1]),4);
			// cout << "temp; " << temp.pose.position.x << ", " << temp.pose.position.x << endl;  
			temp.pose.position.z = 0;

			q.setRPY(0, 0, points[j][2]);

			temp.pose.orientation.x = q.x();
			temp.pose.orientation.y = q.y();
			temp.pose.orientation.z = q.z();
			temp.pose.orientation.w = q.w();
			path_msg.poses.push_back(temp);
		}
		// cout << "=================================\n";
		points.clear();
	}

	// RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path2");

	pub2_->publish(path_msg);
}

void Controller::plotarc(const Dubins_arc& arc, std::vector<std::vector<float>> &points)
{
	std::vector<float> temp;
	temp.push_back(arc.x0);
	temp.push_back(arc.y0);
	temp.push_back(arc.th0);
	points.insert(points.begin(),temp);

	std::vector<std::vector<float>>::iterator row;
	std::vector<float>::iterator col;

	for 
	(int i = 1; i <= no_of_samples; i++)
	{
		float s = (arc.L / no_of_samples) * i;

		float x,y,th = 0.0;

		circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);

		std::vector<float> temp2;

		if
		(!(isnan(x) || isnan(y) || isnan(th)))
		{
			temp2.push_back(x);
			temp2.push_back(y);
			temp2.push_back(th);
		}
		else
		{
			temp2 = points[i-1];
		}
		
		points.insert(points.begin() + i, temp2);

		temp2.clear();
	}
}

void Controller::circline
(float s, float x0, float y0, float th0, float k, float &x, float &y, float &th)
{
	x = x0 + s * sinc(k * s/2.0) * std::cos(th0 + k * s/2.0);
	y = y0 + s * sinc(k * s/2.0) * std::sin(th0 + k * s / 2.0);
	th = mod2pi(th0 + k * s);
}

void Controller::get_robot_pose(const geometry_msgs::msg::TransformStamped msg)
{
	cout << "got robot pose\n";
	// geometry_msgs::msg::Pose ret;
	// // rclcpp::Parameter str_param = this->get_parameter("target_frame");
	// // if (str_param.value_to_string().c_str() == "not set")
	// // {
	// std::string target_frame_ = this->declare_parameter<std::string>("target_frame", "shelfino1/base_link");
	// // }
	// std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
	// std::unique_ptr<tf2_ros::Buffer> tf_buffer;
	// tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	// tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
	// std::string fromFrameRel = target_frame_.c_str();
	// std::string toFrameRel = "map";
	// geometry_msgs::msg::TransformStamped t;
	// try {
	// 		t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 5s);
	// } catch (const tf2::TransformException & ex) {
	// 		RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
	// 		return ret;
	// }
	// tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
	// tf2::Matrix3x3 m(q);
	// double roll, pitch, yaw;
	// m.getRPY(roll, pitch, yaw);
	// geometry_msgs::msg::Quaternion qq;
	// qq.x = q.getX();
	// ret.position.x = t.transform.translation.x;
	// ret.position.y = t.transform.translation.y;
	// ret.orientation = qq;
	// return ret;
}

void Controller::get_border(const geometry_msgs::msg::Polygon msg)
{
	border = msg;
	if (!mapBuilt && msg.points.size() == 4) // ensure that we have a rectangle
	{
		std::vector<boost::geometry::model::d2::point_xy<float>> points;
		for (auto& p : msg.points)
		{
			points.push_back(boost::geometry::model::d2::point_xy<float>(p.x, p.y));
		}
		planner.CreateGridMap(points[0], points[1], points[2], points[3], 1.0);
		planner.PrintGridMap();
		mapBuilt = true;		
	}
}

void Controller::get_gate_poses(const geometry_msgs::msg::PoseArray msg)
{
	// cout << "gate pose sub\n";
	gate_poses = msg;
	// cout << "gate size: " << gate_poses.poses.size() << endl;
	// cout << "x: " << gate_poses.poses[0].position.x << ", y: " << gate_poses.poses[0].position.y << endl;
}

void Controller::get_obs(const obstacles_msgs::msg::ObstacleArrayMsg msg)
{
	// cout << "osb sub\n";
	
	if(obs.size() == 0 && obs_list.obstacles.size() == 0)
	{
		obs_list = msg;

		for 
		(size_t i = 0; i < msg.obstacles.size(); i++)
		{
			ObstacleTypes temp;
			temp.ros_poly = msg.obstacles[i].polygon;
			temp.get_boost_poly();
			obs.push_back(temp);
		}

		remove_obs_in_grid();
	}
	// cout << "obs size: " << obs.size() << endl;
	// cout << "x: " << obs_list.obstacles[0].polygon.points[0].x << ", y: " << obs_list.obstacles[0].polygon.points[0].y << endl;
}

void Controller::remove_obs_in_grid()
{   
    for 
        (size_t i = 0; i < gridmap.content.size(); i++)
    {
        for 
            (size_t j = 0; j < gridmap.content[i].size(); j++)
        {   
            for 
            (size_t k = 0; k < obs.size(); k++)
            {
                if(boost::geometry::covered_by(gridmap.content[i][j], obs[k].boost_poly))
                {
                    cout << "inside obs: " << gridmap.content[i][j].x() << ", " << gridmap.content[i][j].y() << endl;
                    gridmap.content[i][j].x(std::numeric_limits<float>::infinity());
                    gridmap.content[i][j].y(std::numeric_limits<float>::infinity());
                }
            }
        } 
    }  
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}