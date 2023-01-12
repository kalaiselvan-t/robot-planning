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
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "obstacles_msgs/msg/waypoints_msg.hpp"

using namespace std;

typedef boost::geometry::model::d2::point_xy<float> point;
typedef boost::geometry::model::polygon<point> polygon;

// Data Structs

struct Point
{
    float x;
    float y;
    point boost_pt;
    geometry_msgs::msg::Point32 ros_pt;

    Point(geometry_msgs::msg::Point32 inp);

    Point(point inp);

    Point();

    void get_boost_pt();
    void get_ros_pt();

    // bool operator!=(const point& rhs)
    // {
    //     return boost_pt.x() != rhs.x() && boost_pt.y() != rhs.y();
    // }

    bool operator==(const point& rhs){
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
    vector<vector<point>> grid;
    void create_grid();
    void print_grid();
};

struct ObstacleTypes
{
    geometry_msgs::msg::Polygon ros_poly;
    boost::geometry::model::polygon<point> boost_poly;

    geometry_msgs::msg::Polygon boost2ros(boost::geometry::model::polygon<point>);
    boost::geometry::model::polygon<point> ros2boost(geometry_msgs::msg::Polygon);

    void get_boost_poly();
    void get_ros_poly();
};

// void handler(int sig) {
//   cout << "Caught segmentation fault" << endl;
// }

bool operator==(const point& lhs, const point& rhs);


// Initialisations
extern int resolution;
extern int max_border_x;
extern int max_border_y;

extern random_device rd;

extern point pt;
extern Nodes roadmap_nodes;
extern Grid map_grid;
extern vector<ObstacleTypes> obs_list;
extern vector<pair<float,float>> obs_pts_list;

// Function Declarations

float round_up(float inp, int places);

NodeGraph create_random_node();

void test_link();

#endif
