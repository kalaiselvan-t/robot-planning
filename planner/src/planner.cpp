#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "../include/planner/r_map.h"
#include "./dubins_planner.cpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;
using namespace std;

class PlannerNode : public rclcpp::Node
{
    public:
        PlannerNode():Node("Planner_node")
        {
            sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "/inflated_obstacles",1,bind(&PlannerNode::topic_callback, this, _1));

            publisher_ = this->create_publisher<nav_msgs::msg::Path>("dubins_path",10);

            timer_ = this->create_wall_timer(1000ms, std::bind(&PlannerNode::timer_callback, this));
        }

    private:
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
        Roadmap roadmap = Roadmap();
        vector<pair<Point,pair<int,int>>> follow_path;
        vector<Pt> waypoints;
        vector<vector<float>> arc1_pts;
        vector<vector<float>> arc2_pts;
        vector<vector<float>> arc3_pts;

        void topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg);

        void timer_callback();

        void publisher();
};

void PlannerNode::topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg)
{
    cout << "sub working\n";
    if
    (obs_list.size() == 0)
    {
        roadmap.create_poly_list(msg);
    }
}

void PlannerNode::timer_callback()
{
    // follow_path.clear();
    ros_point start, end;
    start.x = 0.0;
    start.y = -1.0;
    end.x = 7.0;
    end.y = 6.0;

    if(obs_list.size() > 0 && map_grid.grid.size() == 0)
    {
        map_grid.create_grid();
        roadmap.remove_obs_in_grid();
        roadmap.dup_grid = map_grid;
        // roadmap.print_obs();
    }
    
    if(obs_list.size() > 0 && map_grid.grid.size())
    {
        if(follow_path.size() == 0)
        {                
            roadmap.find_path(start, end, follow_path);
            for (size_t i = 0; i < follow_path.size(); i++)
            {
                Pt p;
                p.x = follow_path[i].first.boost_pt.x();
                p.y = follow_path[i].first.boost_pt.y();
                waypoints.push_back(p);
            }
            for (size_t i = 0; i < waypoints.size(); i++)
            {
                cout << "x: " << waypoints[i].x << ", y: " << waypoints[i].y << endl;
            }
            // roadmap.print_followpath(follow_path);
            cout << "\n===========End of graph=============\n";
            }
            else
            {
                cout << "follow path already created\n";
            }
    }

    // cout << "wpts size: " << waypoints.size() << endl;

    if(waypoints.size() > 0)
    {
        trajectory_arcs.clear();
        multipoints(waypoints);
    }
    else
    {
        cout << "waypoints size is 0\n";
    }

    cout << "traj arc size: " << trajectory_arcs.size() << endl;

    if(trajectory_arcs.size() > 0)
    {
        publisher();
    }
    else
    {
        cout << "trajectory_arcs size is 0\n";
    }
}

void PlannerNode::publisher()
{
    nav_msgs::msg::Path path_msg;
    tf2::Quaternion q; 

    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";
    
    for 
    (size_t i = 0; i < trajectory_arcs.size(); i++)
    {
        plotarc(&trajectory_arcs[i], arc1_pts);
        geometry_msgs::msg::PoseStamped temp;
        temp.header.stamp = this->get_clock()->now();
        temp.header.frame_id = ' ';

        for 
        (int j = 0; j < no_of_samples; j++)
        {
            temp.pose.position.x = round_up((arc1_pts[j][0]),4);
            temp.pose.position.y = round_up((arc1_pts[j][1]),4);
            // cout << "temp; " << temp.pose.position.x << ", " << temp.pose.position.x << endl;  
            temp.pose.position.z = 0;

            q.setRPY(0, 0, arc1_pts[j][2]);

            temp.pose.orientation.x = q.x();
            temp.pose.orientation.y = q.y();
            temp.pose.orientation.z = q.z();
            temp.pose.orientation.w = q.w();
            path_msg.poses.push_back(temp);
        }
        // cout << "=================================\n";
        arc1_pts.clear();
    }

    RCLCPP_INFO(this->get_logger(), "Publishing: dubins_path");

    publisher_->publish(path_msg);
}


int main(int argc, char * argv[])
{
    // Roadmap r = Roadmap();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}