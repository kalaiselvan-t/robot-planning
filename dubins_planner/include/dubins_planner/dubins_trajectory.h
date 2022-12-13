#include <cmath>
#include <vector>
#include <assert.h>

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

struct point
{
	double x,y,th;
};

extern double X0;
extern double Y0;
extern double Xf;
extern double Yf;
extern double Th0;
extern double Thf;
extern double Kmax;
extern int pidx;

extern int no_waypts;
extern int step;
extern double angle_step;

extern int no_of_samples;

extern dubinscurve_out dubin_curve;
extern point init, final;

extern std::vector<point> best_path;

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

int test();

/*====================================================================
=============================Plot Functions===========================
====================================================================*/

// void plotarc(dubinsarc_out *, int (&)[101][2]);
void plotarc(dubinsarc_out *, std::vector<std::vector<double>> &points);

// void plot_dubins(dubinscurve_out *, int (&)[101][2], int (&)[101][2], int (&)[101][2]);

void plot_dubins(dubinscurve_out *, std::vector<std::vector<int>> &c1, std::vector<std::vector<int>> &c2, std::vector<std::vector<int>> &c3);
