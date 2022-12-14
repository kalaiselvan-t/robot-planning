#include "dubins_planner/dubins_trajectory.h"

// Initial Configuration

bool DEBUG = true;

long double X0 = 0.0;
long double Y0 = 0.0;
long double Th0 = -2.0/3.0 * M_PI;

long double Xf = 3.0;
long double Yf = 0.0;
long double Thf = -M_PI / 3.0;

long double Kmax = 3.0;
int no_of_samples = 100;

int no_waypts = 2;
int step = 6;
long double angle_step = M_PI/step;

// Initialize data structures

point init, final;
dubinscurve_out dubin_curve;
std::vector<point> best_path;

// Do not change

int pidx = 0;
