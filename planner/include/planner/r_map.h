#ifndef R_MAP_H
#define R_MAP_H

// BOOST_BIND_NO_PLACEHOLDERS

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <memory>
#include <limits>
#include <csignal>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "obstacles_msgs/msg/waypoints_msg.hpp"

using namespace std;

typedef geometry_msgs::msg::Point32 ros_point;
typedef geometry_msgs::msg::Polygon ros_polygon;

typedef boost::geometry::model::d2::point_xy<float> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;

// Data Structs

struct Point
{
    ros_point ros_pt;
    boost_point boost_pt;

    Point(ros_point inp);

    Point(boost_point inp);

    Point();

    void get_boost_pt();
    void get_ros_pt();

    bool operator==(const boost_point& rhs){
        return boost_pt.x() == rhs.x() && boost_pt.y() == rhs.y();
    }
};

struct NodeGraph
{
    float x;
    float y;
};

struct Nodes
{
    vector<NodeGraph> list;

    void print()
    {
        for 
            (size_t i = 0; i < list.size(); i++)
        {
            cout << "x: " << list[i].x << " y: " << list[i].y << endl;
        }
    }
};

struct Grid
{
    vector<vector<boost_point>> grid;
    void create_grid();
    void print_grid();
};

struct ObstacleTypes
{
    ros_polygon ros_poly;
    boost_polygon boost_poly;

    ros_polygon boost2ros(boost_polygon);
    boost_polygon ros2boost(ros_polygon);

    void get_boost_poly();
    void get_ros_poly();
};

// void handler(int sig) {
//   cout << "Caught segmentation fault" << endl;
// }

bool operator==(const boost_point& lhs, const boost_point& rhs);


// Initialisations
extern int resolution;
extern float resol;
extern int max_border_x;
extern int max_border_y;

extern random_device rd;

extern boost_point pt;
extern Nodes roadmap_nodes;
extern Grid map_grid;
extern vector<ObstacleTypes> obs_list;
extern vector<pair<float,float>> obs_pts_list;

// Function Declarations

float round_up2(float inp, int places);

NodeGraph create_random_node();

void test_link();

// Class declarations

class Roadmap
{
    public:
        vector<pair<Point,pair<int,int>>> follow_path;
        Grid dup_grid;

        void create_poly_list(const obstacles_msgs::msg::ObstacleArrayMsg & msg);
        void remove_obs_in_grid();
        bool start_condition(ros_point start, ros_point end);
        void find_path(ros_point start, ros_point end, vector<pair<Point,pair<int,int>>> &fp);
        void print_obs();
        void print_obs_pts();
        void print_followpath(vector<pair<Point,pair<int,int>>> &fp);

    private:
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_;
        rclcpp::Publisher<obstacles_msgs::msg::WaypointsMsg>::SharedPtr pub_;

        // vector<pair<Point,pair<int,int>>> follow_path;
        // Grid dup_grid;

        void topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg);

        // bool start_condition(geometry_msgs::msg::Point32 st, geometry_msgs::msg::Point32 en);

        void publisher();

        // void create_poly_list(const obstacles_msgs::msg::ObstacleArrayMsg & msg);

        // void remove_obs_in_grid();

        NodeGraph create_random_node();

        void create_node_graph();

        // void find_path(geometry_msgs::msg::Point32 st, geometry_msgs::msg::Point32 en);

        void up(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void up_right(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void right(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void right_down(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void down(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void down_left(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void left(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        void left_up(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp);

        // void print_obs();

        // void print_followpath();

        // void print_obs_pts();
};

#endif
