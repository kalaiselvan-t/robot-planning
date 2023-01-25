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
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Boost
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/intersection.hpp> 

typedef geometry_msgs::msg::Point32 ros_point;
typedef geometry_msgs::msg::Polygon ros_polygon;

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::box<boost_point> boost_box;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;
typedef boost::geometry::model::multi_polygon<boost_polygon> boost_multi_polygon;

// Structs
struct Pose2d
{
	float x,y,th;
};

struct Robot
{
    int id;
    Pose2d pose_2d;
    geometry_msgs::msg::Pose pose;
    boost_polygon boost_poly;

    Robot();
    void update_pose(geometry_msgs::msg::Transform tf);
    
};

struct GridMap
{
    std::vector<std::vector<boost_point>> content;
    void create();
    void print();
};

struct ObstacleTypes
{
    ros_polygon ros_poly;
    boost_polygon boost_poly;

    ros_polygon boost2ros(boost_polygon);
    boost_polygon ros2boost(ros_polygon);

    boost_polygon get_boost_poly();
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

class Planner
{
    public:
        GridMap dup_gridmap;
        vector<pair<Point,pair<int,int>>> follow_path;
        vector<Pose2d> waypoints;
        vector<Pose2d> best_path;
        vector<Dubins_arc> arcs;
        vector<vector<float>> points;

        Planner() {};
        Planner(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, GridMap gridmap);

        // void create_poly_list(const obstacles_msgs::msg::ObstacleArrayMsg & msg);
        // void remove_obs_in_grid();
        void find_path();
        vector<Dubins_arc> plan();
        void print();
        vector<Dubins_arc> multipoints(const std::vector<Pose2d>& waypoints);
        void get_waypoints();
        void print_waypoints();
        void print_followpath();
        vector<Pose2d> return_waypoints();
        Pose2d find_nearest_grid_pt(Pose2d inp, Pose2d goal);
        Pose2d find_nearest_grid_pt(Pose2d goal);
        // void remove_obs_in_grid();
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

// Functions
boost_multi_polygon gen_robot_polygon(Pose2d inp);
bool check_collision(boost_multi_polygon poly);
// Data
extern geometry_msgs::msg::Polygon border;
extern geometry_msgs::msg::PoseArray gate_poses;
extern obstacles_msgs::msg::ObstacleArrayMsg obs_list;
extern std::vector<ObstacleTypes> obs;
extern geometry_msgs::msg::Pose robot_pose;
extern Robot robot1;
extern Robot robot2;
extern Robot robot3;
extern geometry_msgs::msg::Pose robot1_pose;
extern Pose2d bot_pos;
extern Pose2d bot1_pos;

extern int max_border_x;
extern int max_border_y;
extern int no_of_robots;
extern int step;
extern int no_of_obs;
extern int no_of_samples;
extern int no_of_gates;
extern float angle_step;

extern bool frame_flag;

float round_up(float value, int decimal_places);
float sinc(float t);
float mod2pi(float ang);

#endif