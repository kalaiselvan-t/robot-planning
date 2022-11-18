#include <iostream>
#include <cmath>

										//Helper Functions
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

										//Main Functions
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
		
		if (std::abs(temp2) > 1)
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



	void dubins_shortest_path
	(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
	{
		double sc_th0, sc_thf, sc_Kmax, lambda = 0.0;
		scale_to_standard(x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax, lambda)	;
		double sc_s1, sc_s2, sc_s3 = 0.0;
		bool ok = false;
	}


int main(){
	
										//Problem Data
	double X0, Y0 = 0.0;
	double Xf = 4.0;
	double Yf = 0.0;
	double Th0 = -2.0/3.9 * M_PI;
	double Kmax = 3.0;

}
