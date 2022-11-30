#include <iostream>
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
		// std::cout << "circline x0: " << x0 << std::endl;
		// std::cout << "circline y0: " << y0 << std::endl;
		// std::cout << "circline th0: " << th0 << std::endl;
		// std::cout << "circline s: " << s << std::endl;
		// std::cout << "circline k: " << k << std::endl;
		x = x0 + s * sinc(k * s/2.0) * std::cos(th0 + k * s/2.0);
		y = y0 + s * sinc(k * s/2.0) * std::sin(th0 + k * s / 2.0);
		th = mod2pi(th0 + k * s);
		// std::cout << "circline xf: " << x << std::endl;
		// std::cout << "circline yf: " << y << std::endl;
		// std::cout << "circline thf: " << th << std::endl;
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
		circline(l, x0, y0, th0, k, out->xf, out->yf, out->thf);

		//std::cout << "dubins_arc x0: " << x0 << std::endl;
		//std::cout << "dubins_arc y0: " << y0 << std::endl;
		//std::cout << "dubins_arc th0: " << th0 << std::endl;
		//std::cout << "dubins_arc k: " << k << std::endl;
		//std::cout << "dubins_arc l: " << l << std::endl;
		//std::cout << "dubins_arc xf: " << out->xf << std::endl;
		//std::cout << "dubins_arc yf: " << out->yf << std::endl;
		//std::cout << "dubins_arc thf: " << out->thf << std::endl;
		// std::cout << "==============================================="<< std::endl;

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

		//std::cout << "dubins_curve out.a1.x0: " << out->a1.x0 << std::endl;
		//std::cout << "dubins_curve out.a1.y0: " << out->a1.y0 << std::endl;
		//std::cout << "dubins_curve out.a1.th0: " << out->a1.th0 << std::endl;
		//std::cout << "dubins_curve out.a1.xf: " << out->a1.xf << std::endl;
		//std::cout << "dubins_curve out.a1.yf: " << out->a1.yf << std::endl;
		//std::cout << "dubins_curve out.a1.thf: " << out->a1.thf << std::endl;
		//std::cout << "dubins_curve out.a1.k: " << out->a1.k << std::endl;
		//std::cout << "dubins_curve out.a1.l: " << out->a1.l << std::endl;
		//std::cout << "==============================================="<< std::endl;
		//std::cout << "dubins_curve out.a2.x0: " << out->a2.x0 << std::endl;
                //std::cout << "dubins_curve out.a2.y0: " << out->a2.y0 << std::endl;
                //std::cout << "dubins_curve out.a2.th0: " << out->a2.th0 << std::endl;
                //std::cout << "dubins_curve out.a2.xf: " << out->a2.xf << std::endl;
                //std::cout << "dubins_curve out.a2.yf: " << out->a2.yf << std::endl;
                //std::cout << "dubins_curve out.a2.thf: " << out->a2.thf << std::endl;
                //std::cout << "dubins_curve out.a2.k: " << out->a2.k << std::endl;
                //std::cout << "dubins_curve out.a2.l: " << out->a2.l << std::endl;
                //std::cout << "==============================================="<< std::endl;
		//std::cout << "dubins_curve out.a3.x0: " << out->a3.x0 << std::endl;
                //std::cout << "dubins_curve out.a3.y0: " << out->a3.y0 << std::endl;
                //std::cout << "dubins_curve out.a3.th0: " << out->a3.th0 << std::endl;
                //std::cout << "dubins_curve out.a3.xf: " << out->a3.xf << std::endl;
                //std::cout << "dubins_curve out.a3.yf: " << out->a3.yf << std::endl;
                //std::cout << "dubins_curve out.a3.thf: " << out->a3.thf << std::endl;
                //std::cout << "dubins_curve out.a3.k: " << out->a3.k << std::endl;
                //std::cout << "dubins_curve out.a3.l: " << out->a3.l << std::endl;
                //std::cout << "==============================================="<< std::endl;
		//std::cout << "dubins_curve out.a3: " << out->a3 << std::endl;
		//std::cout << "dubins_curve out.L: " << out->L << std::endl;
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

		// std::cout << "check s1: " << s1 << std::endl;
		// std::cout << "check k0: " << k0 << std::endl;
		// std::cout << "check s2: " << s2 << std::endl;
		// std::cout << "check k1: " << k1 << std::endl;
		// std::cout << "check s3: " << s3 << std::endl;
		// std::cout << "check k2: " << k2 << std::endl;
		// std::cout << "check th0: " << th0 << std::endl;
		// std::cout << "check thf: " << thf << std::endl;

		double eq1 = x0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::cos(th0 + (1.0/2.0) * k0 * s1)
           + s2 * sinc((1.0/2.0) * k1 * s2) * std::cos(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
           + s3 * sinc((1.0/2.0) * k2 * s3) * std::cos(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - xf;

      //std::cout << "check eq1: " << eq1 << std::endl;

      double eq2 = y0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::sin(th0 + (1.0/2.0) * k0 * s1)
           + s2 * sinc((1.0/2.0) * k1 * s2) * std::sin(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
           + s3 * sinc((1.0/2.0) * k2 * s3) * std::sin(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - yf;

      // std::cout << "check eq2: " << eq2 << std::endl;

  		double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

  		// std::cout << "check eq3: " << eq3 << std::endl;

  		bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);

  		// std::cout << "check Lpos: " << Lpos << std::endl;

  		bool out = (std::sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1E-10) && Lpos;

  		// std::cout << "sqrt: " << (std::sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1E-10) << std::endl; 
  		// std::cout << "check out: " << out << std::endl;

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

		// std::cout << "LSL invk: " << invK << std::endl;
		// std::cout << "LSL C: " << C << std::endl;
		// std::cout << "LSL S: " << S << std::endl;
		// std::cout << "LSL temp1: " << temp1 << std::endl;
		// std::cout << "LSL sc_s1: " << sc_s1 << std::endl;
		// std::cout << "LSL sc_s2: " << sc_s2 << std::endl;
		// std::cout << "LSL sc_s3: " << sc_s3 << std::endl;
		// std::cout << "LSL temp2: " << temp2 << std::endl;
		// std::cout << "LSL ok: " << ok << std::endl;
		// std::cout << "==============================================="<< std::endl;

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

		// std::cout << "RSR invk: " << invK << std::endl;
		// std::cout << "RSR C: " << C << std::endl;
		// std::cout << "RSR S: " << S << std::endl;
		// std::cout << "RSR temp1: " << temp1 << std::endl;
		// std::cout << "RSR sc_s1: " << sc_s1 << std::endl;
		// std::cout << "RSR sc_s2: " << sc_s2 << std::endl;
		// std::cout << "RSR sc_s3: " << sc_s3 << std::endl;
		// std::cout << "RSR temp2: " << temp2 << std::endl;
		// std::cout << "RSR ok: " << ok << std::endl;
		// std::cout << "==============================================="<< std::endl;
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

		// std::cout << "LSR invk: " << invK << std::endl;
		// std::cout << "LSR C: " << C << std::endl;
		// std::cout << "LSR S: " << S << std::endl;
		// std::cout << "LSR temp1: " << temp1 << std::endl;
		// std::cout << "LSR sc_s1: " << sc_s1 << std::endl;
		// std::cout << "LSR sc_s2: " << sc_s2 << std::endl;
		// std::cout << "LSR sc_s3: " << sc_s3 << std::endl;
		// std::cout << "LSR temp2: " << temp2 << std::endl;
		// std::cout << "LSR ok: " << ok << std::endl;
		// std::cout << "==============================================="<< std::endl;
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

		// std::cout << "RSL invk: " << invK << std::endl;
		// std::cout << "RSL C: " << C << std::endl;
		// std::cout << "RSL S: " << S << std::endl;
		// std::cout << "RSL temp1: " << temp1 << std::endl;
		// std::cout << "RSL sc_s1: " << sc_s1 << std::endl;
		// std::cout << "RSL sc_s2: " << sc_s2 << std::endl;
		// std::cout << "RSL sc_s3: " << sc_s3 << std::endl;
		// std::cout << "RSL temp2: " << temp2 << std::endl;
		// std::cout << "RSL temp3: " << temp3 << std::endl;
		// std::cout << "RSL ok: " << ok << std::endl;
		// std::cout << "==============================================="<< std::endl;
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
			// std::cout << "Entering loop temp2 > 1" << std::endl;
			ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
			//std::cout << sc_s1 << "," << sc_s2 << "," << sc_s3 << "," << ok << std::endl;
			// std::cout << "RLR invk: " << invK << std::endl;
       	// std::cout << "RLR C: " << C << std::endl;
       	// std::cout << "RLR S: " << S << std::endl;
       	// std::cout << "RLR temp1: " << temp1 << std::endl;
       	// std::cout << "RLR sc_s1: " << sc_s1 << std::endl;
       	// std::cout << "RLR sc_s2: " << sc_s2 << std::endl;
       	// std::cout << "RLR sc_s3: " << sc_s3 << std::endl;
       	// std::cout << "RLR temp2: " << temp2 << std::endl;
       	// std::cout << "RLR ok: " << ok << std::endl;
       	// std::cout << "==============================================="<< std::endl;
			return;
		}
		
		sc_s2 = invK * mod2pi(2.0 * M_PI - std::acos(temp2));
		sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
		sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
		ok = true;

		// std::cout << "RLR invk: " << invK << std::endl;
		// std::cout << "RLR C: " << C << std::endl;
		// std::cout << "RLR S: " << S << std::endl;
		// std::cout << "RLR temp1: " << temp1 << std::endl;
		// std::cout << "RLR sc_s1: " << sc_s1 << std::endl;
		// std::cout << "RLR sc_s2: " << sc_s2 << std::endl;
		// std::cout << "RLR sc_s3: " << sc_s3 << std::endl;
		// std::cout << "RLR temp2: " << temp2 << std::endl;
		// std::cout << "RLR ok: " << ok << std::endl;
		// std::cout << "==============================================="<< std::endl;
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
			// std::cout << "LRL invk: " << invK << std::endl;
         // std::cout << "LRL C: " << C << std::endl;
	      // std::cout << "LRL S: " << S << std::endl;
        	// std::cout << "LRL temp1: " << temp1 << std::endl;
         // std::cout << "LRL sc_s1: " << sc_s1 << std::endl;
	      // std::cout << "LRL sc_s2: " << sc_s2 << std::endl;
         // std::cout << "LRL sc_s3: " << sc_s3 << std::endl;
         // std::cout << "LRL temp2: " << temp2 << std::endl;
   		// std::cout << "LRL ok: " << ok << std::endl;
         // std::cout << "==============================================="<< std::endl;
			return;
		}
		
		sc_s2 = invK * mod2pi(2.0 * M_PI - std::acos(temp2));
		sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
		sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
		ok = true;

		// std::cout << "LRL invk: " << invK << std::endl;
		// std::cout << "LRL C: " << C << std::endl;
		// std::cout << "LRL S: " << S << std::endl;
		// std::cout << "LRL temp1: " << temp1 << std::endl;
		// std::cout << "LRL sc_s1: " << sc_s1 << std::endl;
		// std::cout << "LRL sc_s2: " << sc_s2 << std::endl;
		// std::cout << "LRL sc_s3: " << sc_s3 << std::endl;
		// std::cout << "LRL temp2: " << temp2 << std::endl;
		// std::cout << "LRL ok: " << ok << std::endl;
		// std::cout << "==============================================="<< std::endl;
	}
	

	void scale_to_standard
	(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, double &sc_th0, double &sc_thf, double &sc_Kmax, double &lambda)
	{
		double dx = xf - x0;
		double dy = yf - y0;
		double phi = std::atan2(dy, dx);
		lambda = std::hypot(dx, dy) / 2.0;

		// std::cout << "scale_to_standard dx: " << dx << std::endl;
		// std::cout << "scale_to_standard dy: " << y0 << std::endl;
		// std::cout << "scale_to_standard phi: " << phi << std::endl;
		// std::cout << "scale_to_standard lambda: " << lambda << std::endl;

		sc_th0 = mod2pi(th0 - phi);
		sc_thf = mod2pi(thf - phi);
		sc_Kmax = Kmax * lambda;

		// std::cout << "scale_to_standard sc_th0: " << sc_th0 << std::endl;
		// std::cout << "scale_to_standard sc_thf: " << sc_thf << std::endl;
		// std::cout << "scale_to_standard sc_Kmax: " << sc_Kmax << std::endl;
		// std::cout << "==============================================="<< std::endl;
	}

	void scale_from_standard
	(double lambda, double sc_s1, double sc_s2, double sc_s3, double &s1, double &s2, double &s3)
	{
		s1 = sc_s1 * lambda;
		s2 = sc_s2 * lambda;
		s3 = sc_s3 * lambda;
		// std::cout << "scale_from_standard s1: " << s1 << std::endl;
		// std::cout << "scale_from_standard s2: " << s2 << std::endl;
		// std::cout << "scale_from_standard s3: " << s3 << std::endl;
		// std::cout << "==============================================="<< std::endl;

	}



	void dubins_shortest_path
	(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, int &pidx, dubinscurve_out *curve)
	{
		// std::cout << "dubins_shortest_path x0: " << x0 << std::endl;
		// std::cout << "dubins_shortest_path y0: " << y0 << std::endl;
		// std::cout << "dubins_shortest_path th0: " << th0 << std::endl;
		// std::cout << "dubins_shortest_path xf: " << xf << std::endl;
		// std::cout << "dubins_shortest_path yf: " << yf << std::endl;
		// std::cout << "dubins_shortest_path thf: " << thf << std::endl;
		// std::cout << "dubins_shortest_path kmax: " << Kmax << std::endl;
		// std::cout << "dubins_shortest_path pidx: " << pidx << std::endl;
		// std::cout << "==============================================="<< std::endl;

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

		int ksigns[6][3] = {1, 0, 1, -1, 0, -1, 1, 0, -1, -1, 0, 1, -1, 1, -1, 1, -1, 1};

		pidx = -1;
		double L = std::numeric_limits<double>::max();

		double sc_s1_c, sc_s2_c, sc_s3_c = 0.0;

		double Lcur = 0.0;

		for
		(int i = 0; i < primitives.size(); i++)
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
			}

			//std::cout << "dubins_shortest_path loop " << i << " sc_s1: " << sc_s1 << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " sc_s2: " << sc_s2 << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " sc_s3: " << sc_s3 << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " sc_s1_c: " << sc_s1_c << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " sc_s2_c: " << sc_s2_c << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " sc_s3_c: " << sc_s3_c << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " Lcur: " << Lcur << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " L: " << L << std::endl;
			//std::cout << "dubins_shortest_path loop " << i << " pidx: " << pidx << std::endl;
		}

		// std::cout << "dubins_shortest_path sc_s1: " << sc_s1 << std::endl;
		// std::cout << "dubins_shortest_path sc_s2: " << sc_s2 << std::endl;
		// std::cout << "dubins_shortest_path sc_s3: " << sc_s3 << std::endl;
		// std::cout << "dubins_shortest_path sc_s1_c: " << sc_s1_c << std::endl;
		// std::cout << "dubins_shortest_path sc_s2_c: " << sc_s2_c << std::endl;
		// std::cout << "dubins_shortest_path sc_s3_c: " << sc_s3_c << std::endl;
		// std::cout << "dubins_shortest_path LCur: " << Lcur << std::endl;
		// std::cout << "dubins_shortest_path L: " << L << std::endl;
		// std::cout << "dubins_shortest_path ok: " << ok << std::endl;
		// std::cout << "dubins_shortest_path pidx: " << pidx << std::endl;
		// std::cout << "==============================================="<< std::endl;


		if
		(pidx > 0)
		{
			double s1, s2, s3 = 0.0;
			scale_from_standard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);
			// std::cout << "dubins_shortest_path s1: " << s1 << std::endl;
			// std::cout << "dubins_shortest_path s2: " << s2 << std::endl;
			// std::cout << "dubins_shortest_path s3: " << s3 << std::endl;
			// std::cout << "==============================================="<< std::endl;

			// std::cout << "pidx loop" << std::endl;

			dubins_curve(x0, y0, th0, s1, s2, s3, ksigns[pidx-1][0] * Kmax, ksigns[pidx-1][1] * Kmax, ksigns[pidx-1][2] * Kmax, curve);

			assert(check(sc_s1, ksigns[pidx-1][0] * sc_Kmax, sc_s2, ksigns[pidx-1][1] * sc_Kmax, sc_s3, ksigns[pidx-1][2] * sc_Kmax, sc_th0, sc_thf));

			// std::cout << "inside loop pidx: " << pidx << std::endl;
		}
	}


int main(){
	
										//Problem Data
	double X0, Y0 = 0.0;
	double Xf = 4.0;
	double Yf = 0.0;
	double Th0 = -2.0/3.0 * M_PI;
	Th0 = Th0, 4;
	double Thf = M_PI / 3.0;
	Thf = Thf, 4;
	double Kmax = 3.0;

	// std::cout << "Main function x0: " << X0 << std::endl;
	// std::cout << "Main function y0: " << Y0 << std::endl;
	// std::cout << "Main function th0: " << Th0 << std::endl;
	// std::cout << "Main function xf: " << Xf << std::endl;
	// std::cout << "Main function yf: " << Yf << std::endl;
	// std::cout << "Main function thf: " << Thf << std::endl;
	// std::cout << "Main function kmax: " << Kmax << std::endl;
	// std::cout << "==============================================="<< std::endl;

	int pidx = 0;
	dubinscurve_out dubin_curve;

	dubins_shortest_path(X0, Y0, Th0, Xf, Yf, Thf, Kmax, pidx, &dubin_curve);

	std::cout << "pidx: " << pidx << "\n";

}
