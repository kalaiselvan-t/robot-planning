#include "../include/controller/common.hpp"
#include "../include/controller/data.hpp"

// Initializations
geometry_msgs::msg::Polygon border;
geometry_msgs::msg::PoseArray gate_poses;
obstacles_msgs::msg::ObstacleArrayMsg obs_list;
geometry_msgs::msg::Pose robot_pose;
Robot robot1 = Robot();
Robot robot2 = Robot();
Robot robot3 = Robot();
geometry_msgs::msg::Pose robot1_pose;
Pose2d bot_pos;
Pose2d bot1_pos;
std::vector<ObstacleTypes> obs;

int max_border_x = -1;
int max_border_y = -1;
int no_of_robots = 1;
int step = 20;
int no_of_samples = 100;
int no_of_gates = -1;
int no_of_obs = -1;

float angle_step = 2*M_PI/step;

bool frame_flag = false;