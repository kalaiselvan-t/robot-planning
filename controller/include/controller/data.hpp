#ifndef DATA
#define DATA

// Geometry msgs
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Boost
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

typedef geometry_msgs::msg::Point32 ros_point;
typedef geometry_msgs::msg::Polygon ros_polygon;

typedef boost::geometry::model::d2::point_xy<float> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;

// Structs
struct Pose2d
{
	float x,y,th;
};

struct Grid
{
    std::vector<std::vector<boost_point>> grid;
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

struct Point
{
    geometry_msgs::msg::Pose ros_pose;
    boost_point boost_pt;

    Point(geometry_msgs::msg::Pose inp);

    Point(boost_point inp);

    Point();

    void get_boost_pt();
    void get_ros_pose();

    bool operator==(const boost_point& rhs){
        return boost_pt.x() == rhs.x() && boost_pt.y() == rhs.y();
    }
};

struct Dubins_arc{
        float x0;
        float y0;
        float th0;
        float k;
        float L;
        float xf;
        float yf;
        float thf;
    void print(){
        cout<<"x0: "<<x0<<"\n";
        cout<<"y0: "<<y0<<"\n";
        cout<<"th0: "<<th0<<"\n";
        cout<<"k: "<<k<<"\n";
        cout<<"L: "<<L<<"\n";
        cout<<"xf: "<<xf<<"\n";
        cout<<"yf: "<<yf<<"\n";
        cout<<"thf: "<<thf<<"\n";
    }
};

// Class

class Planner
{
    public:
        Grid dup_grid;
        vector<pair<Point,pair<int,int>>> follow_path;
        vector<Pose2d> waypoints;
        vector<Pose2d> best_path;
        vector<Dubins_arc> arcs;
        vector<vector<float>> points;

        Planner(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, Grid map);

        // void create_poly_list(const obstacles_msgs::msg::ObstacleArrayMsg & msg);
        // void remove_obs_in_grid();
        // bool start_condition(ros_point start, ros_point end);
        void find_path();
        vector<Dubins_arc> plan();
        void print();
        vector<Dubins_arc> multipoints();
        void get_waypoints();
        void print_waypoints();
        void print_followpath();
        // void print_obs_pts();
        // void print_followpath(vector<pair<Point,pair<int,int>>> &fp);

    private:

        geometry_msgs::msg::Pose start;

        geometry_msgs::msg::Pose end;

        void up(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void up_right(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void right(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void right_down(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void down(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void down_left(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void left(Point end, pair<Point,pair<int,int>> &best,float &best_dist);

        void left_up(Point end, pair<Point,pair<int,int>> &best,float &best_dist);
};

// Data
extern geometry_msgs::msg::Polygon border;
extern geometry_msgs::msg::PoseArray gate_poses;
extern obstacles_msgs::msg::ObstacleArrayMsg obs_list;
extern std::vector<ObstacleTypes> obs;
extern geometry_msgs::msg::Pose robot_pose;

extern int max_border_x;
extern int max_border_y;
extern int no_of_robots;
extern int step;
extern int no_of_samples;
extern float angle_step;

extern bool frame_flag;

float round_up(float value, int decimal_places);
float sinc(float t);
float mod2pi(float ang);

#endif