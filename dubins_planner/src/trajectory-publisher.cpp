#include <iostream>
#include <cmath>
#include <vector>

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
	===========================Helper functions===========================
	====================================================================*/

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
		x = x0 + s * sinc(k * s/2.0) * std::cos(th0 + k * s / 2.0);
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

		out->xf, out->yf, out->thf = 0.0;
		circline(x0, y0, th0, k, l, out->xf, out->yf, out->thf);
	}

	void dubins_curve
	(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2, dubinscurve_out *out)
	{
		dubinsarc_out a1, a2, a3;
		dubins_arc(x0, y0, th0, k0, s1, &a1);
		dubins_arc(x0, y0, th0, k0, s1, &a2);
		dubins_arc(x0, y0, th0, k0, s1, &a3);
		out->L = a1.l + a2.l + a3.l;
		out->a1 = a1;
		out->a2 = a2;
		out->a3 = a3;
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
	(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
	{
		double sc_th0, sc_thf, sc_Kmax, lambda = 0.0;
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

		int pidx = -1;
		int L = std::numeric_limits<int>::max();

		double sc_s1_c, sc_s2_c, sc_s3_c = 0.0;

		for
		(int i = 0; i < primitives.size(); i++)
		{
			primitives[i](sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c, ok);
			double Lcur = sc_s1 + sc_s2 + sc_s3;

			if
			(ok && Lcur < L)
			{
				L = Lcur;
				sc_s1 = sc_s1_c;
				sc_s2 = sc_s2_c;
				sc_s3 = sc_s3_c;
				pidx = i;

			}
		}

		//curve[];

		if
		(pidx > 0)
		{
			double s1, s2, s3 = 0.0;
			scale_from_standard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);
		}
	}


int main(){
	
										//Problem Data
	double X0, Y0 = 0.0;
	double Xf = 4.0;
	double Yf = 0.0;
	double Th0 = -2.0/3.9 * M_PI;
	double Kmax = 3.0;

	std::cout << Xf << " " << Kmax << "\n";

}
