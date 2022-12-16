#include "dubins_planner/dubins_trajectory.h"

// Initial Configuration

bool DEBUG = true;

float X0 = 0.0;
float Y0 = 0.0;
float Th0 = -2.0/3.0 * M_PI;

float Xf = -3.0;
float Yf = 0.0;
float Thf = -M_PI / 3.0;

float Kmax = 3.0;
int no_of_samples = 100;

int no_waypts = 3;
int step = 12;
float angle_step = 2*M_PI/step;

// Initialize data structures

point init, final;
dubinscurve_out dubin_curve;
std::vector<point> best_path;
std::vector<dubinscurve_out> trajectory_points;
std::vector<dubinsarc_out> trajectory_arcs;

// Do not change

int pidx = 0;
