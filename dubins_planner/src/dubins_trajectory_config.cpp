#include "dubins_planner/dubins_trajectory.h"

// Initial Configuration

bool DEBUG = false;

double X0 = 0.0;
double Y0 = 0.0;
double Th0 = round_up((-2.0/3.0 * M_PI),4);

double Xf = 5.0;
double Yf = 0.0;
double Thf = round_up((-M_PI / 3.0),4);

double Kmax = 3.0;
int no_of_samples = 100;

int no_waypts = 3;
int step = 6;
double angle_step = M_PI/step;

// Initialize data structures

point init, final;
dubinscurve_out dubin_curve;
std::vector<point> best_path;

// Do not change

int pidx = 0;
