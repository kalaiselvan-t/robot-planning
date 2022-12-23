#include "dubins_planner/dubins_trajectory.h"
#include "utils/utils.h"

/*============================================
===================dubins=====================
=============================================*/

// Initial Configuration

bool DEBUG = true;

float X0 = 0.0;
float Y0 = -2.0;
float Th0 = -2.0/3.0 * M_PI;

float Xf = 2.0;
float Yf = 5.0;
float Thf = M_PI / 3.0;

float Kmax = 3.0;
int no_of_samples = 100;

int no_waypts = 11;
int step = 20;
float angle_step = 2*M_PI/step;

// Initialize data structures

point init, final;
dubinscurve_out dubin_curve;
std::vector<point> best_path;
std::vector<dubinscurve_out> trajectory_points;
std::vector<dubinsarc_out> trajectory_arcs;
std::vector<point> waypoints = {
	{0.0,0.0,0.0},
	{2.0,0.0,0.0},
	{4.0,0.0,0.0},
	{8.0,0.0,0.0},
	{8.0,2.0,0.0},
	{6.0,2.0,0.0},
	{4.0,2.0,0.0},
	{2.0,2.0,0.0},
	{0.0,2.0,0.0},
    // {0.0,4.0,0.0},
};
// Do not change

int pidx = 0;

/*============================================
==================obstacles===================
=============================================*/

std::vector<std::vector<std::vector<int>>> obstacle_list = {{{6,3},{11,3},{11,4},{6,4}}, {{4,12},{9,13},{6,16}},{{2,8},{3,7},{5,11},{6,10}},{{12,7},{15,7},{15,10}}};

