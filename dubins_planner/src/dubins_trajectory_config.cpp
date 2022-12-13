#include "dubins_planner/dubins_trajectory.h"

// Initial Configuration

double X0 = 5.0;

double Y0 = 5.0;

double Xf = 8.0;

double Yf = 8.0;

// double Th0 = round_up((-2.0/3.0 * M_PI),4);

double Th0 = -2.0944;

// double Thf = round_up((-M_PI / 3.0),4);

double Thf = -1.0472;

double Kmax = 3.0;

int no_of_samples = 100;


int no_waypts = 2;

int step = 6;

double angle_step = M_PI/step;


point init, final;

dubinscurve_out dubin_curve;

std::vector<point> best_path;

// Do not change

int pidx = 0;