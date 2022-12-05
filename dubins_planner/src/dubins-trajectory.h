#include <cmath>
#include <vector>
#include <assert.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

/*====================================================================
==========================Data structures=============================
====================================================================*/
struct dubinsarc_out
{
	double x0, y0, th0, k, l, xf, yf, thf;
};

struct dubinscurve_out
{
	dubinsarc_out a1;
	dubinsarc_out a2;
	dubinsarc_out a3;
	double L;
};

/*====================================================================
============================Helper functions==========================
====================================================================*/

double round_up(double, int);

double sinc(double);

double mod2pi(double);

void circline(double, double, double, double, double, double &, double &, double &);

void dubins_arc(double, double, double, double, double, dubinsarc_out *);

void dubins_curve(double, double, double, double, double, double, double, double, double, dubinscurve_out *);

double rangeSymm(double);

bool check(double, double, double, double, double, double, double, double);

/*====================================================================
=========================Function Declarations========================
====================================================================*/

void LSL(double, double, double, double &, double &, double &, bool &);

void RSR(double, double, double, double &, double &, double &, bool &);	

void LSR(double, double, double, double &, double &, double &, bool &);

void RSL(double, double, double, double &, double &, double &, bool &);
	
void RLR(double, double, double, double &, double &, double &, bool &);

void LRL(double, double, double, double &, double &, double &, bool &);

void scale_to_standard(double, double, double, double, double, double, double, double &, double &, double &, double &);

void scale_from_standard(double, double, double, double, double &, double &, double &);

void dubins_shortest_path(double, double, double, double, double, double, double, int &, dubinscurve_out *);

