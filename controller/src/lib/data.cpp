#include "../../include/controller/common.hpp"
#include "../../include/controller/data.hpp"

// Initializations
geometry_msgs::msg::Polygon border;
geometry_msgs::msg::PoseArray gate_poses;
obstacles_msgs::msg::ObstacleArrayMsg obs_list;
geometry_msgs::msg::Pose robot_pose;
std::vector<ObstacleTypes> obs;

int max_border_x = 20;
int max_border_y = 20;
int no_of_robots = 1;
int step = 20;
int no_of_samples = 100;

float angle_step = 2*M_PI/step;

bool frame_flag = false;