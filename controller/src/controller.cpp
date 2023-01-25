#include "rclcpp/rclcpp.hpp"
#include "../include/controller/common.hpp"
#include "../include/controller/data.hpp"
#include "../include/controller/a_star.hpp"

using std::placeholders::_1;

bool operator==(const boost_point& lhs, const Pose2d& rhs)
{
    return lhs.x() == rhs.x && lhs.y() == rhs.y;
}

bool operator==(const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose& rhs)
{
    return lhs.position.x == rhs.position.x && lhs.position.y == lhs.position.y;
}

class Controller: public rclcpp::Node
{
    public:
        Controller() : Node("controller_node")
        {
			auto border_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
			border_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "map_borders", border_qos, std::bind(&Controller::get_border, this, _1));

			auto gate_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
	  		gate_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "gate_position", gate_qos, std::bind(&Controller::get_gate_poses, this, _1));

	  		obs_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
				"inflated_obstacles", 1, std::bind(&Controller::get_obs, this, _1));

			mapBuilt = false;
			pathSent = false;
			obstaclesSet = false;
			auto qos_robot1_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
	  		robot1_pose_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/shelfino1/transform", qos_robot1_pose, std::bind(&Controller::get_robot1_pose, this, _1));

			auto qos_robot2_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
	  		robot2_pose_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/shelfino2/transform", qos_robot2_pose, std::bind(&Controller::get_robot2_pose, this, _1));

	  		auto qos_robot3_pose = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
			robot3_pose_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/shelfino3/transform", qos_robot3_pose, std::bind(&Controller::get_robot3_pose, this, _1));

	  		pub1_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan",10);

			pub2_ = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan",10);

	  		timer_ = this->create_wall_timer(
      			1000ms, std::bind(&Controller::timer_callback, this));

        }
    
    private:
		rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr border_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_poses_sub_;
		rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obs_sub_;
		rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot1_pose_sub_;
		rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot2_pose_;
		rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot3_pose_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub2_;
		rclcpp::TimerBase::SharedPtr timer_;

		AStarPlanner planner;
		bool mapBuilt;
		bool pathSent;
		bool obstaclesSet;
		
		size_t count_;
		GridMap gridmap;
		GridMap map;
		GridMap map1;
		bool map_updated = false;

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
		void update_map(GridMap &map,vector<Pose2d> inp);
		void circline(float s, float x0, float y0, float th0, float k, float &x, float &y, float &th);
		void get_robot1_pose(const geometry_msgs::msg::TransformStamped msg);
		void get_robot2_pose(const geometry_msgs::msg::TransformStamped msg);
		void get_robot3_pose(const geometry_msgs::msg::TransformStamped msg);
		void print_map(GridMap map);
		void find_nearest_grid_pt(Pose2d inp, Pose2d goal);
		bool start_condition();
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
	// nav_msgs::msg::Path path_msg;
	cout << "\n===============Timer===============\n";

	// Pose2d p;
	// p.x = robot_pose.position.x;
	// p.y = robot_pose.position.y;
	// boost_multi_polygon robot_poly = gen_robot_polygon(p);
	// bool ok = check_collision(robot_poly);
	// cout << "Robot collision: " << ok << endl;

	// Pose2d p;
	// p.x = 3.96;
	// p.y = 4.86;

	// Pose2d g;
	// g.x = -3.96;
	// g.y = 4.86;

	// find_nearest_grid_pt(p, g);
	// print_map(this->map);
	// bool ok = start_condition();
	// if(gate_poses.poses.size() > 0 && max_border_x > 0 && no_of_obs > 0 && ok)
	// {
	// 	geometry_msgs::msg::Pose p;
	// 	p.position.x = 2.0;
	// 	p.position.y = 1.0;
	// 	Planner planner = Planner(p,gate_poses.poses[0],this->map);
	// 	this->path1_arcs = planner.plan();
	// 	vector<Pose2d> wpts1 = planner.return_waypoints();
	// 	wpts1.pop_back();
	// 	for(size_t i = 0; i < wpts1.size(); i++)
	// 	{
	// 		cout << "=====================\n";
	// 		cout << "wpts x: " << wpts1[i].x << ", wpts y: " << wpts1[i].y << endl;
	// 		cout << "=====================\n";
	// 	}
	// 	// this->map1.grid.clear();
	// 	// this->map1 = this->map;
	// 	// update_map(this->map1, wpts1);
	// 	// map_updated = true;
	// 	// path1_publisher();
	// 	// Planner planner1 = Planner(robot2.pose,gate_poses.poses[1],this->map);
	// 	// this->path2_arcs = planner1.plan();
	// 	// path2_publisher();
	// }
	// else
	// {
	// 	if(!ok)
	// 	{
	// 		cout << "Robot or end point is inside obstacle\n";
	// 	}
	// 	else if(gate_poses.poses.size() == 0)
	// 	{
	// 		"Gate pose not received\n";
	// 	}
	// 	else if(max_border_x == -1)
	// 	{
	// 		"Borders not received\n";
	// 	}
	// 	else if(no_of_obs == 0)
	// 	{
	// 		"Obstacles not received\n";
	// 	}
	// }

	// cout << "border x: " << map_border_x << ", y: " << map_border_y << endl;
	// geometry_msgs::msg::Pose rp;
	// geometry_msgs::msg::Pose ep;
	// rp.position.x = 0.0;
	// rp.position.y = 0.0;
	// ep.position.x = 5.0;
	// ep.position.y = 7.0;
	// geometry_msgs::msg::Pose goal_pose = gate_poses.poses.size();
	// cout << "x: " << gate_poses.poses.size() << endl;
	
	// if(gate_poses.poses.size() > 0)
	// {
	// 	Planner planner = Planner(robot_pose,gate_poses.poses[0],this->map);
	// 	this->path1_arcs = planner.plan();
	// 	vector<Pose2d> wpts1 = planner.return_waypoints();
	// 	wpts1.pop_back();
	// 	// for(size_t i = 0; i < wpts1.size(); i++)
	// 	// {
	// 	// 	cout << "=====================\n";
	// 	// 	cout << "wpts x: " << wpts1[i].x << ", wpts y: " << wpts1[i].y << endl;
	// 	// 	cout << "=====================\n";
	// 	// }
	// 	this->map1.grid.clear();
	// 	this->map1 = this->map;
	// 	update_map(this->map1, wpts1);
	// 	map_updated = true;
	// 	path1_publisher();
	// }
	// else
	// {
	// 	cout << "gate pos not yet received\n";
	// }

	// print_map(map2);
	// geometry_msgs::msg::Pose rp1;
	// geometry_msgs::msg::Pose ep1;
	// rp1.position.x = 2.0;
	// rp1.position.y = 2.0;
	// ep1.position.x = 5.0;
	// ep1.position.y = 7.0;
	// Planner planner1 = Planner(rp1,ep1,this->map);
	// this->path2_arcs = planner1.plan();
	// path_msg = path2_publisher();
	
	// wpts1.clear();
	// this->map1.grid.clear();
}

void Controller::update_map(GridMap &map, vector<Pose2d> inp)
{
	cout << "size i: " << map.content.size() << endl;
	for (size_t i = 0; i < map.content.size(); i++)
	{
		// cout << "j: " << this->map.grid[i].size() << endl;
		for (size_t j = 0; j < map.content[i].size(); j++)
		{
			// cout << "==========================\n";

			// for(size_t l = 0; l < obs.size(); l++)
            // {
            //     if(boost::geometry::covered_by(map.grid[i][j], obs[l].boost_poly))
            //     {
            //         cout << "inside obs: " << map.grid[i][j].x() << ", " << map.grid[i][j].y() << endl;
            //         map.grid[i][j].x(std::numeric_limits<float>::infinity());
            //         map.grid[i][j].y(std::numeric_limits<float>::infinity());
            //     }
            // }

			for (size_t k = 0; k < inp.size(); k++)
			{
				// 
				if(map.content[i][j].x() == inp[k].x && map.content[i][j].y() == inp[k].y)
				{
					map.content[i][j].x(std::numeric_limits<float>::infinity());
					map.content[i][j].y(std::numeric_limits<float>::infinity());
				}
			}
			// cout << "==========================\n";
		}	
	}
	
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

	RCLCPP_INFO(this->get_logger(), "Publishing: shelfino1/plan");

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

	RCLCPP_INFO(this->get_logger(), "Publishing: shelfino2/plan");

	pub2_->publish(path_msg);
}


void Controller::get_robot1_pose(const geometry_msgs::msg::TransformStamped msg)
{
	// cout << "\n==========Robot1 Pose Start==========\n";
	geometry_msgs::msg::Transform tf;
    tf = msg.transform;

	robot1.update_pose(tf);
	// cout << "robot1 x: " << robot1.pose_2d.x << ", y: " << robot1.pose_2d.y << ", th: " << robot1.pose_2d.th << endl;
	// cout << "\n==========Robot1 Pose End==========\n";
}

void Controller::get_robot2_pose(const geometry_msgs::msg::TransformStamped msg)
{
	// cout << "\n==========Robot2 Pose Start==========\n";
	geometry_msgs::msg::Transform tf;
    tf = msg.transform;

	robot2.update_pose(tf);
	// cout << "robot2 x: " << robot2.pose_2d.x << ", y: " << robot2.pose_2d.y << ", th: " << robot2.pose_2d.th << endl;
	// cout << "\n==========Robot2 Pose End==========\n";
}

void Controller::get_robot3_pose(const geometry_msgs::msg::TransformStamped msg)
{
	// cout << "\n==========Robot3 Pose Start==========\n";
	geometry_msgs::msg::Transform tf;
    tf = msg.transform;

	robot3.update_pose(tf);
	// cout << "robot3 x: " << robot3.pose_2d.x << ", y: " << robot3.pose_2d.y << ", th: " << robot3.pose_2d.th << endl;
	// cout << "\n==========Robot3 Pose End==========\n";
}

void Controller::get_border(const geometry_msgs::msg::Polygon msg)
{
	geometry_msgs::msg::Polygon border;
	border = msg;
	float max_x = 0.0;
	float min_x = 0.0;
	float max_y = 0.0;
	float min_y = 0.0;
	// cout << "==========Border Start==========\n";
	// cout << "border x: " << max_border_x << ", border y: " << max_border_y << endl;
	for(size_t i = 0; i < border.points.size(); i++)
	{	
		if(border.points[i].x < min_x)
		{
			min_x = border.points[i].x;
		}
		else if(border.points[i].x > max_x)
		{
			max_x = border.points[i].x;
		}

		if(border.points[i].y < min_y)
		{
			min_y = border.points[i].y;
		}
		else if(border.points[i].y > max_y)
		{
			max_y = border.points[i].y;
		}
		// cout << "x: " << border.points[i].x << ", y: " << border.points[i].y << endl;
	}
	max_border_x = abs(min_x) + abs(max_x);
	max_border_y = abs(min_y) + abs(min_y);
	this->map.create();
	remove_obs_in_grid();
	// cout << "min x: " << min_x << ",max_x: " << max_x << "\nmin_y: " << min_y << ",max_y: " << max_y << endl;
	// cout << "==========Border End==========\n";

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
	// cout << "==========Gate Poses Start==========\n";
	gate_poses = msg;
	no_of_gates = gate_poses.poses.size();
	// cout << "gate size: " << gate_poses.poses.size() << endl;
	// for(size_t i = 0; i < gate_poses.poses.size(); i++)
	// {
	// 	cout << "x: " << gate_poses.poses[i].position.x << ", y: " << gate_poses.poses[i].position.y << endl;
	// }
	// cout << "==========Gate Poses End==========\n";
}

void Controller::get_obs(const obstacles_msgs::msg::ObstacleArrayMsg msg)
{
	if (mapBuilt && !obstaclesSet && msg.obstacles.size() > 0) // ensure that we have polygons
	{
		std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>> obstacles;
		for (auto& obs : msg.obstacles)
		{
			ObstacleTypes temp;
			temp.ros_poly = obs.polygon;
			obstacles.push_back(temp.get_boost_poly());
			// auto p = ObstaclesTypes::ros2boost(polygon);
			// obstacles.push_back(p);
		}
		// planner.CreateGridMap(points[0], points[1], points[2], points[3], 1.0);
		// planner.PrintGridMap();
		planner.SetObstacles(obstacles);
		obstaclesSet = true;
	}

	// cout << "==========Obs Start==========\n";
	// no_of_obs = obs_list.obstacles.size();
	// // cout << "no of obs: " << no_of_obs << endl;
	// if(obs.size() == 0 && obs_list.obstacles.size() == 0)
	// {
	// 	obs_list = msg;
	// 	for 
	// 	(size_t i = 0; i < msg.obstacles.size(); i++)
	// 	{
	// 		ObstacleTypes temp;
	// 		temp.ros_poly = msg.obstacles[i].polygon;
	// 		temp.get_boost_poly();
	// 		obs.push_back(temp);
	// 	}

	// 	remove_obs_in_grid();
	// }
	// cout << "==========Obs End==========\n";
	// cout << "obs size: " << obs.size() << endl;
	// cout << "x: " << obs_list.obstacles[0].polygon.points[0].x << ", y: " << obs_list.obstacles[0].polygon.points[0].y << endl;
}

void Controller::remove_obs_in_grid()
{   
    for(size_t i = 0; i < this->map.content.size(); i++)
    {
        for(size_t j = 0; j < this->map.content[i].size(); j++)
        {   
            for(size_t k = 0; k < obs.size(); k++)
            {
                if(boost::geometry::covered_by(this->map.content[i][j], obs[k].boost_poly))
                {
                    // cout << "inside obs: " << this->map.grid[i][j].x() << ", " << this->map.grid[i][j].y() << endl;
                    this->map.content[i][j].x(std::numeric_limits<float>::infinity());
                    this->map.content[i][j].y(std::numeric_limits<float>::infinity());
                }
            }
        } 
    }  
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

void Controller::find_nearest_grid_pt(Pose2d inp, Pose2d goal)
{
	vector<pair<float,float>> list;
	pair<float, float> p;
	float best_dist = std::numeric_limits<float>::infinity();
	Pose2d best_pt;
	boost_point goal_pt;
	goal_pt.x(goal.x);
	goal_pt.y(goal.y);

	// floor floor
	p.first = floor(inp.x);
	p.second = floor(inp.y);
	list.push_back(p);

	// floor ceil
	p.first = floor(inp.x);
	p.second = ceil(inp.y);
	list.push_back(p);

	// ceil floor
	p.first = ceil(inp.x);
	p.second = floor(inp.y);
	list.push_back(p);

	// ceil ceil
	p.first = ceil(inp.x);
	p.second = ceil(inp.y);
	list.push_back(p);

	for(size_t i = 0; i < list.size(); i++)
	{
		boost_point bp;
		bp.x(list[i].first);
		bp.y(list[i].second);
		float dist = boost::geometry::distance(bp,goal_pt);
		if(dist < best_dist)
		{
			best_dist = dist;
			best_pt.x = list[i].first;
			best_pt.y = list[i].second;
		}
		// cout << "distance to goal: " << boost::geometry::distance(bp,goal) << endl;
	}
	// cout << "best pt: " << "x: " << best_pt.x << ", y: " << best_pt.y << endl;
}

void Controller::print_map(GridMap map)
{
	cout << "=======================MAP START==========================\n";
	for(int i = 0; i < map.content.size(); i++)
    {
        for(int j = 0; j < map.content[i].size(); j++)
        {   
            cout << "x: " << map.content[i][j].x() << ", y: " << map.content[i][j].y() << endl;
        } 
    }
	cout << "=======================MAP END==========================\n";
}

bool Controller::start_condition()
{
    if(no_of_obs > 0)
	{
		for(size_t k = 0; k < obs.size(); k++)
		{
			if(boost::geometry::covered_by(boost_point(robot1.pose_2d.x,robot1.pose_2d.y), obs[k].boost_poly))
			{
				return false;
			}
		}
		return true;
	}
	else
	{
		cout << "obs data not available\n";
		return true;
	}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}