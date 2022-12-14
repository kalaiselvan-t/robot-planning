#include <iostream>
#include "dubins_planner/dubins_trajectory.h"

/*====================================================================
===========================Helper functions===========================
====================================================================*/

double round_up
(double value, int decimal_places)
{
	const double multiplier = std::pow(10.0, decimal_places);
	return std::ceil(value * multiplier) / multiplier;
}

double sinc
(double t)
{
	double out;
	if
	(std::abs(t) < 0.002)
	{
		out = 1 - std::pow(t,2.0/6.0) * (1 - std::pow(t,2.0/20.0));
	}
	else
	{
		out = std::sin(t) / t;
	}

	return out;
}

double mod2pi
(double ang)
{
	double out = ang;
	while (out < 0)
	{
		out = out + (2.0 * M_PI);
	}
	while (out >= 2.0 * M_PI)
	{
		out = out - (2.0 * M_PI);
	}
	return out;
}

void circline
(double s, double x0, double y0, double th0, double k, double &x, double &y, double &th)
{
	x = x0 + s * sinc(k * s/2.0) * std::cos(th0 + k * s/2.0);
	y = y0 + s * sinc(k * s/2.0) * std::sin(th0 + k * s / 2.0);
	th = mod2pi(th0 + k * s);
}

void dubins_arc
(double x0, double y0, double th0, double k, double l, dubinsarc_out *out)
{
	out->x0 = x0;
	out->y0 = y0;
	out->th0 = th0;
	out->k = k;
	out->l = l;

	out->xf = 0.0;
	out->yf = 0.0;
	out->thf = 0.0;
	circline(l, x0, y0, th0, k, out->xf, out->yf, out->thf);

}

void dubins_curve
(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2, dubinscurve_out *out)
{
	dubinsarc_out a1, a2, a3;
	dubins_arc(x0, y0, th0, k0, s1, &a1);
	dubins_arc(a1.xf, a1.yf, a1.thf, k1, s2, &a2);
	dubins_arc(a2.xf, a2.yf, a2.thf, k2, s3, &a3);
	out->L = a1.l + a2.l + a3.l;
	out->a1 = a1;
	out->a2 = a2;
	out->a3 = a3;
}

double rangeSymm
(double ang)
{
	double out = ang;

	while
	(out <= -M_PI)
	{
		out = out + 2 * M_PI;
	}

	while
	(out > M_PI)
	{
		out = out - 2 * M_PI;
	}

	return out;
}

bool check
(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf)
{
	int x0 = -1;
	int y0 = 0;
	int xf = 1;
	int yf = 0;

	double eq1 = x0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::cos(th0 + (1.0/2.0) * k0 * s1)
       + s2 * sinc((1.0/2.0) * k1 * s2) * std::cos(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
       + s3 * sinc((1.0/2.0) * k2 * s3) * std::cos(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - xf;

  	double eq2 = y0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::sin(th0 + (1.0/2.0) * k0 * s1)
       + s2 * sinc((1.0/2.0) * k1 * s2) * std::sin(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
       + s3 * sinc((1.0/2.0) * k2 * s3) * std::sin(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - yf;

	double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

	bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);

	bool out = (std::sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1E-10) && Lpos;

	return out;
}

/*==================================================================
=============================Functions==============================
==================================================================*/

void LSL
(double sc_th0, double sc_thf, double sc_Kmax, double &sc_s1, double &sc_s2, double &sc_s3, bool &ok)
{
	double invK = 1.0 / sc_Kmax;
	double C = std::cos(sc_thf) - std::cos(sc_th0);
	double S = 2.0 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);
	double temp1 = std::atan2(C, S);
	sc_s1 = invK * mod2pi(temp1 - sc_th0);
	double temp2 = 2.0 + 4.0 * std::pow(sc_Kmax, 2) - 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp2 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1);
	ok = true;

}

	
void RSR
(double sc_th0, double sc_thf, double sc_Kmax, double &sc_s1, double &sc_s2, double &sc_s3, bool &ok)
{
	double invK = 1.0 / sc_Kmax;
	double C = std::cos(sc_th0) - std::cos(sc_thf);
	double S = 2.0 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);
	double temp1 = std::atan2(C, S);
	sc_s1 = invK * mod2pi(sc_th0 - temp1);
	double temp2 = 2.0 + 4.0 * std::pow(sc_Kmax, 2) - 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp2 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp2);
	sc_s3 = invK * mod2pi(temp1 - sc_thf);
	ok = true;
}	


void LSR
(double sc_th0, double sc_thf, double sc_Kmax, double &sc_s1, double &sc_s2, double &sc_s3, bool &ok)
{
	double invK = 1.0 / sc_Kmax;
	double C = std::cos(sc_th0) + std::cos(sc_thf);
	double S = 2.0 * sc_Kmax + std::sin(sc_th0) + std::sin(sc_thf);
	double temp1 = std::atan2(-C, S);
	double temp3 = 4.0 * std::pow(sc_Kmax, 2) - 2.0 + 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp3 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp3);
	double temp2 = -std::atan2(-2.0, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
	sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
	ok = true;
}	


void RSL
(double sc_th0, double sc_thf, double sc_Kmax, double &sc_s1, double &sc_s2, double &sc_s3, bool &ok)
{
	double invK = 1.0 / sc_Kmax;
	double C = std::cos(sc_th0) + std::cos(sc_thf);
	double S = 2.0 * sc_Kmax - std::sin(sc_th0) - std::sin(sc_thf);
	double temp1 = std::atan2(C, S);
	double temp3 = 4.0 * std::pow(sc_Kmax, 2) - 2.0 + 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	
	if (temp3 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp3);
	double temp2 = std::atan2(2.0, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
	ok = true;
}	

	
void RLR
(double sc_th0, double sc_thf, double sc_Kmax, double &sc_s1, double &sc_s2, double &sc_s3, bool &ok)
{
	double invK = 1.0 / sc_Kmax;
	double C = std::cos(sc_th0) - std::cos(sc_thf);
	double S = 2.0 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);
	double temp1 = std::atan2(C, S);
	double temp2 = 0.125 * (6.0 - 4.0 * std::pow(sc_Kmax, 2) + 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf))); 
	
	if (std::abs(temp2) > 1)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
		return;
	}
	
	sc_s2 = invK * mod2pi(2.0 * M_PI - std::acos(temp2));
	sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
	sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
	ok = true;
}


void LRL
(double sc_th0, double sc_thf, double sc_Kmax, double &sc_s1, double &sc_s2, double &sc_s3, bool &ok)
{
	double invK = 1.0 / sc_Kmax;
	double C = std::cos(sc_thf) - std::cos(sc_th0);
	double S = 2.0 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);
	double temp1 = std::atan2(C, S);
	double temp2 = 0.125 * (6.0 - 4.0 * std::pow(sc_Kmax,2) + 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf))); 
	
	if 
	(std::abs(temp2) > 1)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
		return;
	}
	
	sc_s2 = invK * mod2pi(2.0 * M_PI - std::acos(temp2));
	sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
	sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
	ok = true;
}


void scale_to_standard
(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, double &sc_th0, double &sc_thf, double &sc_Kmax, double &lambda)
{
	double dx = xf - x0;
	double dy = yf - y0;
	double phi = std::atan2(dy, dx);
	lambda = std::hypot(dx, dy) / 2.0;

	sc_th0 = mod2pi(th0 - phi);
	sc_thf = mod2pi(thf - phi);
	sc_Kmax = Kmax * lambda;
}

void scale_from_standard
(double lambda, double sc_s1, double sc_s2, double sc_s3, double &s1, double &s2, double &s3)
{
	s1 = sc_s1 * lambda;
	s2 = sc_s2 * lambda;
	s3 = sc_s3 * lambda;
}

void dubins_shortest_path
(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, int &pidx, dubinscurve_out *curve)
{

	double sc_th0, sc_thf, sc_Kmax, lambda = 0.0;

	if
	(DEBUG)
	{
		std::cout << "dubins_shortest_path\n";
		std::cout << "============================\n";
		std::cout << "x0: " << x0 << ", " << "y0: " << y0 << ", " << "th0: " << th0 << ", " << "xf: " << xf << ", " << "yf: " << yf << ", " << "thf: " << thf << std::endl;
		std::cout << "Kmax: " << Kmax << ", " << "pidx: " << pidx << std::endl;
	}

	scale_to_standard(x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax, lambda)	;

	double sc_s1, sc_s2, sc_s3 = 0.0;
	bool ok = false;
																																		//Function pointers
	void (*lsl)(double, double, double, double&, double&, double&, bool&);
	void (*rsr)(double, double, double, double&, double&, double&, bool&);
	void (*lsr)(double, double, double, double&, double&, double&, bool&);
	void (*rsl)(double, double, double, double&, double&, double&, bool&);
	void (*rlr)(double, double, double, double&, double&, double&, bool&);
	void (*lrl)(double, double, double, double&, double&, double&, bool&);

																																		//Functions vector
	std::vector<void (*)(double, double, double, double&, double&, double&, bool&)> primitives;

																																		//Initialization
	lsl = &LSL;
	rsr = &RSR;
	lsr = &LSR;
	rsl = &RSL;
	rlr = &RLR;
	lrl = &LRL;

	primitives.push_back(lsl);
	primitives.push_back(rsr);
	primitives.push_back(lsr);
	primitives.push_back(rsl);
	primitives.push_back(rlr);
	primitives.push_back(lrl);

	int ksigns[6][3] = {1, 0, 1, -1, 0, -1, 1, 0, -1, -1, 0, 1, -1, 1, -1, 1, -1, 1};

	pidx = -1;
	double L = std::numeric_limits<double>::max();

	double sc_s1_c, sc_s2_c, sc_s3_c = 0.0;

	double Lcur = 0.0;

	for
	(size_t i = 0; i < primitives.size(); i++)
	{
		primitives[i](sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c, ok);
		Lcur = sc_s1_c + sc_s2_c + sc_s3_c;

		if
		(ok && Lcur < L)
		{
			L = Lcur;
			sc_s1 = sc_s1_c;
			sc_s2 = sc_s2_c;
			sc_s3 = sc_s3_c;
			pidx = i+1;
		}		}

	if
	(pidx > 0)
	{
		double s1, s2, s3 = 0.0;
		scale_from_standard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);

		dubins_curve(x0, y0, th0, s1, s2, s3, ksigns[pidx-1][0] * Kmax, ksigns[pidx-1][1] * Kmax, ksigns[pidx-1][2] * Kmax, curve);

		// assert(check(sc_s1, ksigns[pidx-1][0] * sc_Kmax, sc_s2, ksigns[pidx-1][1] * sc_Kmax, sc_s3, ksigns[pidx-1][2] * sc_Kmax, sc_th0, sc_thf));

	}
}

/*====================================================================
=============================Plot Functions===========================
====================================================================*/

void plotarc
(dubinsarc_out *arc, std::vector<std::vector<double>> &points)
{
	std::vector<double> temp;
	temp.push_back(arc->x0);
	temp.push_back(arc->y0);
	temp.push_back(arc->th0);
	points.insert(points.begin(),temp);

	std::vector<std::vector<double>>::iterator row;
	std::vector<double>::iterator col;

	for 
	(int i = 1; i <= no_of_samples; i++)
	{
		double s = arc->l / no_of_samples * i;

		double x,y,th = 0.0;

		circline(s, arc->x0, arc->y0, arc->th0, arc->k, x, y, th);

		std::vector<double> temp2;

		temp2.push_back(x);
		temp2.push_back(y);
		temp2.push_back(th);

		points.insert(points.begin() + i, temp2);

		temp2.clear();
	}

	if
	(DEBUG)
	{
		for 
		(int i = 0; i < no_of_samples; ++i)
		{
			std::cout << points[i][0] << ", " << points[i][1] << std::endl; 
		}
		std::cout << "======================================\n";
	}

}

void plot_dubins
(dubinscurve_out *curve, std::vector<std::vector<double>> &c1, std::vector<std::vector<double>> &c2, std::vector<std::vector<double>> &c3)
{
	plotarc(&curve->a1, c1);
	plotarc(&curve->a2, c2);
	plotarc(&curve->a3, c3);

}