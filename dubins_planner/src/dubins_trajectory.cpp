#include <iostream>
#include "dubins_planner/dubins_trajectory.h"

/*====================================================================
===========================Helper functions===========================
====================================================================*/

float round_up
(float value, int decimal_places)
{
	const float multiplier = std::pow(10.0, decimal_places);
	return std::ceil(value * multiplier) / multiplier;
}

float sinc
(float t)
{
	float out;
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

float mod2pi
(float ang)
{
	float out = ang;
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
(float s, float x0, float y0, float th0, float k, float &x, float &y, float &th)
{
	x = x0 + s * sinc(k * s/2.0) * std::cos(th0 + k * s/2.0);
	y = y0 + s * sinc(k * s/2.0) * std::sin(th0 + k * s / 2.0);
	th = mod2pi(th0 + k * s);
}

void dubins_arc
(float x0, float y0, float th0, float k, float l, dubinsarc_out *out)
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
(float x0, float y0, float th0, float s1, float s2, float s3, float k0, float k1, float k2, dubinscurve_out *out)
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

float rangeSymm
(float ang)
{
	float out = ang;

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
(float s1, float k0, float s2, float k1, float s3, float k2, float th0, float thf)
{
	int x0 = -1;
	int y0 = 0;
	int xf = 1;
	int yf = 0;

	float eq1 = x0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::cos(th0 + (1.0/2.0) * k0 * s1)
       + s2 * sinc((1.0/2.0) * k1 * s2) * std::cos(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
       + s3 * sinc((1.0/2.0) * k2 * s3) * std::cos(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - xf;

  	float eq2 = y0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::sin(th0 + (1.0/2.0) * k0 * s1)
       + s2 * sinc((1.0/2.0) * k1 * s2) * std::sin(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
       + s3 * sinc((1.0/2.0) * k2 * s3) * std::sin(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - yf;

	float eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

	bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);

	bool out = (std::sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1E-10) && Lpos;

	return out;
}

/*==================================================================
=============================Functions==============================
==================================================================*/

void LSL
(float sc_th0, float sc_thf, float sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3, bool &ok)
{
	float invK = 1.0 / sc_Kmax;
	float C = std::cos(sc_thf) - std::cos(sc_th0);
	float S = 2.0 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);
	float temp1 = std::atan2(C, S);
	sc_s1 = invK * mod2pi(temp1 - sc_th0);
	float temp2 = 2.0 + 4.0 * std::pow(sc_Kmax, 2) - 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp2 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1);
	ok = true;

}

	
void RSR
(float sc_th0, float sc_thf, float sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3, bool &ok)
{
	float invK = 1.0 / sc_Kmax;
	float C = std::cos(sc_th0) - std::cos(sc_thf);
	float S = 2.0 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);
	float temp1 = std::atan2(C, S);
	sc_s1 = invK * mod2pi(sc_th0 - temp1);
	float temp2 = 2.0 + 4.0 * std::pow(sc_Kmax, 2) - 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp2 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp2);
	sc_s3 = invK * mod2pi(temp1 - sc_thf);
	ok = true;
}	


void LSR
(float sc_th0, float sc_thf, float sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3, bool &ok)
{
	float invK = 1.0 / sc_Kmax;
	float C = std::cos(sc_th0) + std::cos(sc_thf);
	float S = 2.0 * sc_Kmax + std::sin(sc_th0) + std::sin(sc_thf);
	float temp1 = std::atan2(-C, S);
	float temp3 = 4.0 * std::pow(sc_Kmax, 2) - 2.0 + 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	
	if (temp3 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp3);
	float temp2 = -std::atan2(-2.0, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
	sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
	ok = true;
}	


void RSL
(float sc_th0, float sc_thf, float sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3, bool &ok)
{
	float invK = 1.0 / sc_Kmax;
	float C = std::cos(sc_th0) + std::cos(sc_thf);
	float S = 2.0 * sc_Kmax - std::sin(sc_th0) - std::sin(sc_thf);
	float temp1 = std::atan2(C, S);
	float temp3 = 4.0 * std::pow(sc_Kmax, 2) - 2.0 + 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	
	if (temp3 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp3);
	float temp2 = std::atan2(2.0, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
	ok = true;
}	

	
void RLR
(float sc_th0, float sc_thf, float sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3, bool &ok)
{
	float invK = 1.0 / sc_Kmax;
	float C = std::cos(sc_th0) - std::cos(sc_thf);
	float S = 2.0 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);
	float temp1 = std::atan2(C, S);
	float temp2 = 0.125 * (6.0 - 4.0 * std::pow(sc_Kmax, 2) + 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf))); 
	
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
(float sc_th0, float sc_thf, float sc_Kmax, float &sc_s1, float &sc_s2, float &sc_s3, bool &ok)
{
	float invK = 1.0 / sc_Kmax;
	float C = std::cos(sc_thf) - std::cos(sc_th0);
	float S = 2.0 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);
	float temp1 = std::atan2(C, S);
	float temp2 = 0.125 * (6.0 - 4.0 * std::pow(sc_Kmax,2) + 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf))); 
	
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
(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax, float &sc_th0, float &sc_thf, float &sc_Kmax, float &lambda)
{
	float dx = xf - x0;
	float dy = yf - y0;
	float phi = std::atan2(dy, dx);
	lambda = std::hypot(dx, dy) / 2.0;

	sc_th0 = mod2pi(th0 - phi);
	sc_thf = mod2pi(thf - phi);
	sc_Kmax = Kmax * lambda;
}

void scale_from_standard
(float lambda, float sc_s1, float sc_s2, float sc_s3, float &s1, float &s2, float &s3)
{
	s1 = sc_s1 * lambda;
	s2 = sc_s2 * lambda;
	s3 = sc_s3 * lambda;
}

void dubins_shortest_path
(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax, int &pidx, dubinscurve_out *curve)
{

	float sc_th0, sc_thf, sc_Kmax, lambda = 0.0;

	// if
	// (DEBUG)
	// {
	// 	std::cout << "dubins_shortest_path\n";
	// 	std::cout << "============================\n";
	// 	std::cout << "x0: " << x0 << ", " << "y0: " << y0 << ", " << "th0: " << th0 << ", " << "xf: " << xf << ", " << "yf: " << yf << ", " << "thf: " << thf << std::endl;
	// 	std::cout << "Kmax: " << Kmax << ", " << "pidx: " << pidx << std::endl;
	// }

	scale_to_standard(x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax, lambda)	;

	float sc_s1, sc_s2, sc_s3 = 0.0;
	bool ok = false;
																																		//Function pointers
	void (*lsl)(float, float, float, float&, float&, float&, bool&);
	void (*rsr)(float, float, float, float&, float&, float&, bool&);
	void (*lsr)(float, float, float, float&, float&, float&, bool&);
	void (*rsl)(float, float, float, float&, float&, float&, bool&);
	void (*rlr)(float, float, float, float&, float&, float&, bool&);
	void (*lrl)(float, float, float, float&, float&, float&, bool&);

																																		//Functions vector
	std::vector<void (*)(float, float, float, float&, float&, float&, bool&)> primitives;

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
	float L = std::numeric_limits<float>::max();

	float sc_s1_c, sc_s2_c, sc_s3_c = 0.0;

	float Lcur = 0.0;

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
		float s1, s2, s3 = 0.0;
		scale_from_standard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);

		dubins_curve(x0, y0, th0, s1, s2, s3, ksigns[pidx-1][0] * Kmax, ksigns[pidx-1][1] * Kmax, ksigns[pidx-1][2] * Kmax, curve);

		// assert(check(sc_s1, ksigns[pidx-1][0] * sc_Kmax, sc_s2, ksigns[pidx-1][1] * sc_Kmax, sc_s3, ksigns[pidx-1][2] * sc_Kmax, sc_th0, sc_thf));

	}
}

/*====================================================================
=============================Plot Functions===========================
====================================================================*/

void plotarc
(dubinsarc_out *arc, std::vector<std::vector<float>> &points)
{
	std::vector<float> temp;
	temp.push_back(arc->x0);
	temp.push_back(arc->y0);
	temp.push_back(arc->th0);
	points.insert(points.begin(),temp);

	std::vector<std::vector<float>>::iterator row;
	std::vector<float>::iterator col;

	for 
	(int i = 1; i <= no_of_samples; i++)
	{
		float s = (arc->l / no_of_samples) * i;

		float x,y,th = 0.0;

		circline(s, arc->x0, arc->y0, arc->th0, arc->k, x, y, th);

		std::vector<float> temp2;

		if
		(!(std::isnan(x) || std::isnan(y) || std::isnan(th)))
		{
			temp2.push_back(x);
			temp2.push_back(y);
			temp2.push_back(th);
		}
		else
		{
			temp2 = points[i-1];
		}
		

		points.insert(points.begin() + i, temp2);

		temp2.clear();
	}

	// if
	// (DEBUG)
	// {
	// 	for 
	// 	(int i = 0; i < no_of_samples; ++i)
	// 	{
	// 		std::cout << points[i][0] << ", " << points[i][1] << std::endl; 
	// 	}
	// 	std::cout << "======================================\n";
	// }

}

void plot_dubins
(dubinscurve_out *curve, std::vector<std::vector<float>> &c1, std::vector<std::vector<float>> &c2, std::vector<std::vector<float>> &c3)
{
	plotarc(&curve->a1, c1);
	plotarc(&curve->a2, c2);
	plotarc(&curve->a3, c3);

}