#include <cmath>
#include <vector>
#include <assert.h>

/*====================================================================
==========================Data structures=============================
====================================================================*/
struct dubinsarc_out
{
	float x0, y0, th0, k, l, xf, yf, thf;
};

struct dubinscurve_out
{
	dubinsarc_out a1;
	dubinsarc_out a2;
	dubinsarc_out a3;
	float L;
};

struct point
{
	float x,y,th;
};

/*====================================================================
================================Data==================================
====================================================================*/

extern bool DEBUG;

extern float X0;
extern float Y0;
extern float Xf;
extern float Yf;
extern float Th0;
extern float Thf;
extern float Kmax;
extern int pidx;

extern int no_waypts;
extern int step;
extern float angle_step;

extern int no_of_samples;

extern dubinscurve_out dubin_curve;
extern point init, final;

extern std::vector<point> best_path;
extern std::vector<dubinscurve_out> trajectory_points;
extern std::vector<dubinsarc_out> trajectory_arcs;

/*====================================================================
============================Helper functions==========================
====================================================================*/

float round_up(float, int);

float sinc(float);

float mod2pi(float);

void circline(float, float, float, float, float, float &, float &, float &);

void dubins_arc(float, float, float, float, float, dubinsarc_out *);

void dubins_curve(float, float, float, float, float, float, float, float, float, dubinscurve_out *);

float rangeSymm(float);

bool check(float, float, float, float, float, float, float, float);

/*====================================================================
=========================Function Declarations========================
====================================================================*/

void LSL(float, float, float, float &, float &, float &, bool &);

void RSR(float, float, float, float &, float &, float &, bool &);	

void LSR(float, float, float, float &, float &, float &, bool &);

void RSL(float, float, float, float &, float &, float &, bool &);
	
void RLR(float, float, float, float &, float &, float &, bool &);

void LRL(float, float, float, float &, float &, float &, bool &);

void scale_to_standard(float, float, float, float, float, float, float, float &, float &, float &, float &);

void scale_from_standard(float, float, float, float, float &, float &, float &);

void dubins_shortest_path(float, float, float, float, float, float, float, int &, dubinscurve_out *);

/*====================================================================
=============================Plot Functions===========================
====================================================================*/

void plotarc(dubinsarc_out *, std::vector<std::vector<float>> &points);

void plot_dubins(dubinscurve_out *, std::vector<std::vector<int>> &c1, std::vector<std::vector<int>> &c2, std::vector<std::vector<int>> &c3);
