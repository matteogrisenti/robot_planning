#include <iostream>
#include "dubins_planner/dubins_trajectory.h"

/*====================================================================
===========================Helper functions===========================
====================================================================*/

long double round_up
(long double value, int decimal_places)
{
	const long double multiplier = std::pow(10.0, decimal_places);
	return std::ceil(value * multiplier) / multiplier;
}

long double sinc
(long double t)
{
	long double out;
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

long double mod2pi
(long double ang)
{
	long double out = ang;
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
(long double s, long double x0, long double y0, long double th0, long double k, long double &x, long double &y, long double &th)
{
	x = x0 + s * sinc(k * s/2.0) * std::cos(th0 + k * s/2.0);
	y = y0 + s * sinc(k * s/2.0) * std::sin(th0 + k * s / 2.0);
	th = mod2pi(th0 + k * s);
}

void dubins_arc
(long double x0, long double y0, long double th0, long double k, long double l, dubinsarc_out *out)
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

	if
	(DEBUG)
	{
		std::cout << "\ndubins_arc\n";
		std::cout << "x0: " << x0 << ", " << " y0: " << y0 << ", " << " th0: " << th0 << " k: " << k << ", " << " l: " << l << std::endl;
		std::cout << "xf: " << out->xf << ", " << " yf: " << out->yf << ", " << " thf: " << out->thf << std::endl;  
	}

}

void dubins_curve
(long double x0, long double y0, long double th0, long double s1, long double s2, long double s3, long double k0, long double k1, long double k2, dubinscurve_out *out)
{
	if
	(DEBUG)
	{
		std::cout << "\ndubins_curve\n";
		std::cout << "x0: " << x0 << ", " << " y0: " << y0 << ", " << " th0: " << th0 << " s1: " << s1 << ", " << " s2: " << s2 << ", " << " s3: " << s3 << std::endl;
		std::cout << "k0: " << k0 << ", " << " k1: " << k1 << ", " << " k2: " << k2 << std::endl;  
	}
	dubinsarc_out a1, a2, a3;
	dubins_arc(x0, y0, th0, k0, s1, &a1);
	dubins_arc(a1.xf, a1.yf, a1.thf, k1, s2, &a2);
	dubins_arc(a2.xf, a2.yf, a2.thf, k2, s3, &a3);
	out->L = a1.l + a2.l + a3.l;
	out->a1 = a1;
	out->a2 = a2;
	out->a3 = a3;

	if
	(DEBUG)
	{
		std::cout << "curve.l: " << out->L << std::endl;
	}
}

long double rangeSymm
(long double ang)
{
	long double out = ang;

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
(long double s1, long double k0, long double s2, long double k1, long double s3, long double k2, long double th0, long double thf)
{
	int x0 = -1;
	int y0 = 0;
	int xf = 1;
	int yf = 0;

	long double eq1 = x0 + s1 * sinc((0.5000) * k0 * s1) * std::cos(th0 + (0.5000) * k0 * s1)
       + s2 * sinc((0.5000) * k1 * s2) * std::cos(th0 + k0 * s1 + (0.5000) * k1 * s2)
       + s3 * sinc((0.5000) * k2 * s3) * std::cos(th0 + k0 * s1 + k1 * s2 + (0.5000) * k2 * s3) - xf;

    // long double eq1 = round_up((s1 * sinc((0.5000) * k0 * s1)),4) * round_up((std::cos((1.0/2.0) * k0 * s1)),4); //x0 + s1 * sinc(0.5000 * k0 * s1) * 

  	long double eq2 = y0 + s1 * sinc((1.0/2.0) * k0 * s1) * std::sin(th0 + (1.0/2.0) * k0 * s1)
       + s2 * sinc((1.0/2.0) * k1 * s2) * std::sin(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
       + s3 * sinc((1.0/2.0) * k2 * s3) * std::sin(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - yf;

	long double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

	bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);

	if
	(DEBUG)
	{
		std::cout << "eq1: " << eq1 << " eq2: " << eq2 << " eq3: " << eq3 << " Lpos: " << Lpos << std::endl;
	}

	bool out = (std::sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1E-10) && Lpos;

	if
	(DEBUG)
	{
		std::cout << "out: " << out << std::endl;
	}

	return out;
}

/*==================================================================
=============================Functions==============================
==================================================================*/

void LSL
(long double sc_th0, long double sc_thf, long double sc_Kmax, long double &sc_s1, long double &sc_s2, long double &sc_s3, bool &ok)
{
	long double invK = 1.0 / sc_Kmax;
	long double C = std::cos(sc_thf) - std::cos(sc_th0);
	long double S = 2.0 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);
	long double temp1 = std::atan2(C, S);
	sc_s1 = invK * mod2pi(temp1 - sc_th0);
	long double temp2 = 2.0 + 4.0 * std::pow(sc_Kmax, 2) - 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp2 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1);
	ok = true;

}

	
void RSR
(long double sc_th0, long double sc_thf, long double sc_Kmax, long double &sc_s1, long double &sc_s2, long double &sc_s3, bool &ok)
{
	long double invK = 1.0 / sc_Kmax;
	long double C = std::cos(sc_th0) - std::cos(sc_thf);
	long double S = 2.0 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);
	long double temp1 = std::atan2(C, S);
	sc_s1 = invK * mod2pi(sc_th0 - temp1);
	long double temp2 = 2.0 + 4.0 * std::pow(sc_Kmax, 2) - 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	
	if (temp2 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp2);
	sc_s3 = invK * mod2pi(temp1 - sc_thf);
	ok = true;
}	


void LSR
(long double sc_th0, long double sc_thf, long double sc_Kmax, long double &sc_s1, long double &sc_s2, long double &sc_s3, bool &ok)
{
	long double invK = 1.0 / sc_Kmax;
	long double C = std::cos(sc_th0) + std::cos(sc_thf);
	long double S = 2.0 * sc_Kmax + std::sin(sc_th0) + std::sin(sc_thf);
	long double temp1 = std::atan2(-C, S);
	long double temp3 = 4.0 * std::pow(sc_Kmax, 2) - 2.0 + 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	
	if (temp3 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = round_up((invK * std::sqrt(temp3)),4);
	long double temp2 = round_up((-std::atan2(-2.0, sc_s2 * sc_Kmax)),4);
	sc_s1 = round_up((invK * mod2pi(temp1 + temp2 - sc_th0)),4);
	sc_s3 = round_up((invK * mod2pi(temp1 + temp2 - sc_thf)),4);
	ok = true;

	if
	(DEBUG)
	{
		std::cout << "\nEntering LSR Primitive" << std::endl;
		std::cout << "sc_th0: " << sc_th0 << " sc_thf: " << sc_thf << " sc_Kmax " << sc_Kmax << std::endl;
		std::cout << "invk: " << invK << " C: " << C << " S: " << S << std::endl;
		std::cout << "temp1: " << temp1 << " temp2: " << temp2 << " temp3: " << temp3 << std::endl;
		std::cout << "sc_s1: " << sc_s1 << " sc_s2: " << sc_s2 << " sc_s3: " << sc_s3 << " ok: " << ok << std::endl;
	}
}	


void RSL
(long double sc_th0, long double sc_thf, long double sc_Kmax, long double &sc_s1, long double &sc_s2, long double &sc_s3, bool &ok)
{
	long double invK = 1.0 / sc_Kmax;
	long double C = std::cos(sc_th0) + std::cos(sc_thf);
	long double S = 2.0 * sc_Kmax - std::sin(sc_th0) - std::sin(sc_thf);
	long double temp1 = std::atan2(C, S);
	long double temp3 = 4.0 * std::pow(sc_Kmax, 2) - 2.0 + 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	
	if (temp3 < 0)
	{
		ok = false;sc_s1 = 0.0; sc_s2 = 0.0; sc_s3 = 0.0;
	}
	
	sc_s2 = invK * std::sqrt(temp3);
	long double temp2 = std::atan2(2.0, sc_s2 * sc_Kmax);
	sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
	sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
	ok = true;
}	

	
void RLR
(long double sc_th0, long double sc_thf, long double sc_Kmax, long double &sc_s1, long double &sc_s2, long double &sc_s3, bool &ok)
{
	long double invK = 1.0 / sc_Kmax;
	long double C = std::cos(sc_th0) - std::cos(sc_thf);
	long double S = 2.0 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);
	long double temp1 = std::atan2(C, S);
	long double temp2 = 0.125 * (6.0 - 4.0 * std::pow(sc_Kmax, 2) + 2.0 * std::cos(sc_th0 - sc_thf) + 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf))); 
	
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
(long double sc_th0, long double sc_thf, long double sc_Kmax, long double &sc_s1, long double &sc_s2, long double &sc_s3, bool &ok)
{
	long double invK = 1.0 / sc_Kmax;
	long double C = std::cos(sc_thf) - std::cos(sc_th0);
	long double S = 2.0 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);
	long double temp1 = std::atan2(C, S);
	long double temp2 = 0.125 * (6.0 - 4.0 * std::pow(sc_Kmax,2) + 2.0 * std::cos(sc_th0 - sc_thf) - 4.0 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf))); 
	
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
(long double x0, long double y0, long double th0, long double xf, long double yf, long double thf, long double Kmax, long double &sc_th0, long double &sc_thf, long double &sc_Kmax, long double &lambda)
{
	long double dx = xf - x0;
	long double dy = yf - y0;
	long double phi = std::atan2(dy, dx);
	lambda = std::hypot(dx, dy) / 2.0;

	sc_th0 = round_up((mod2pi(th0 - phi)),4);
	sc_thf = round_up((mod2pi(thf - phi)),4);
	sc_Kmax = round_up((Kmax * lambda),4);
}

void scale_from_standard
(long double lambda, long double sc_s1, long double sc_s2, long double sc_s3, long double &s1, long double &s2, long double &s3)
{
	s1 = sc_s1 * lambda;
	s2 = sc_s2 * lambda;
	s3 = sc_s3 * lambda;
}

void dubins_shortest_path
(long double x0, long double y0, long double th0, long double xf, long double yf, long double thf, long double Kmax, int &pidx, dubinscurve_out *curve)
{

	long double sc_th0, sc_thf, sc_Kmax, lambda = 0.0;

	if
	(DEBUG)
	{
		std::cout << "dubins_shortest_path: Input\n";
		std::cout << "============================\n";
		std::cout << "x0: " << x0 << ", " << "y0: " << y0 << ", " << "th0: " << th0 << ", " << "xf: " << xf << ", " << "yf: " << yf << ", " << "thf: " << thf << std::endl;
		std::cout << "Kmax: " << Kmax << ", " << "pidx: " << pidx << std::endl;
		std::cout << "===============================================================\n";
	}

	scale_to_standard(x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax, lambda);

	if
	(DEBUG)
	{
		std::cout << "\ndubins_shortest_path: scale_to_standard\n";
		std::cout << "============================\n";
		std::cout << "sc_th0: " << sc_th0 << ", " << "sc_thf: " << sc_thf << ", " << "sc_Kmax: " << sc_Kmax << ", " << "lambda: " << lambda << ", " << std::endl;
		std::cout << "===============================================================\n";
	}

	long double sc_s1, sc_s2, sc_s3 = 0.0;
	bool ok = false;
																																		//Function pointers
	void (*lsl)(long double, long double, long double, long double&, long double&, long double&, bool&);
	void (*rsr)(long double, long double, long double, long double&, long double&, long double&, bool&);
	void (*lsr)(long double, long double, long double, long double&, long double&, long double&, bool&);
	void (*rsl)(long double, long double, long double, long double&, long double&, long double&, bool&);
	void (*rlr)(long double, long double, long double, long double&, long double&, long double&, bool&);
	void (*lrl)(long double, long double, long double, long double&, long double&, long double&, bool&);

																																		//Functions vector
	std::vector<void (*)(long double, long double, long double, long double&, long double&, long double&, bool&)> primitives;

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
	long double L = std::numeric_limits<long double>::max();

	long double sc_s1_c, sc_s2_c, sc_s3_c = 0.0;

	long double Lcur = 0.0;

	if
	(DEBUG)
	{
		std::cout << std::endl << "\nEntering loop of primitives" << std::endl;
	}

	for
	(size_t i = 0; i < primitives.size(); i++)
	{
		if
		(DEBUG)
		{
			std::cout << "\nindex: " << i << std::endl;
		}

		primitives[i](sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c, ok);
		Lcur = sc_s1_c + sc_s2_c + sc_s3_c;

		if
		(ok && Lcur < L)
		{
			L = round_up((Lcur),4);
			sc_s1 = round_up((sc_s1_c),4);
			sc_s2 = round_up((sc_s2_c),4);
			sc_s3 = round_up((sc_s3_c),4);
			pidx = i;

			if
			(DEBUG)
			{
				std::cout << "Length: " << L << std::endl;
				std::cout << "sc_s1: " << sc_s1 << std::endl;
				std::cout << "sc_s2: " << sc_s2 << std::endl;
				std::cout << "sc_s3: " << sc_s3 << std::endl;
			}
			
		}		
	}

	if
	(DEBUG)
	{
		std::cout << std::endl << "Finishing loop of primitives" << std::endl;
		std::cout << "pidx: " << pidx << std::endl;
	}

	if
	(pidx >= 0)  // BUG FIX: pidx can be 0 (LSL primitive)
	{
		long double s1, s2, s3 = 0.0;
		scale_from_standard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);

		if
		(DEBUG)
		{
			std::cout << "\ns1: " << s1 << std::endl;
			std::cout << "s2: " << s2 << std::endl;
			std::cout << "s3: " << s3 << std::endl;
		}

		dubins_curve(x0, y0, th0, s1, s2, s3, ksigns[pidx][0] * Kmax, ksigns[pidx][1] * Kmax, ksigns[pidx][2] * Kmax, curve);

		if
		(DEBUG)
		{
			std::cout << "Printing curve details" << std::endl;
			std::cout << "==========\n";
			std::cout << "Arc a1" << std::endl;
			std::cout << "a1.x0: " << curve->a1.x0 << ", " << "a1.y0: " << curve->a1.y0 << ", " << "a1.th0: " << curve->a1.th0 << std::endl;
			std::cout << "a1.xf: " << curve->a1.xf << ", " << "a1.yf: " << curve->a1.yf << ", " << "a1.thf: " << curve->a1.thf << std::endl;
			std::cout << "a1.k: " << curve->a1.k << ", " << "a1.l: " << curve->a1.l << std::endl;
			std::cout << "==========\n";
			std::cout << "Arc a2" << std::endl;
			std::cout << "a2.x0: " << curve->a2.x0 << ", " << "a2.y0: " << curve->a2.y0 << ", " << "a2.th0: " << curve->a2.th0 << std::endl;
			std::cout << "a2.xf: " << curve->a2.xf << ", " << "a2.yf: " << curve->a2.yf << ", " << "a2.thf: " << curve->a2.thf << std::endl;
			std::cout << "a2.k: " << curve->a2.k << ", " << "a2.l: " << curve->a2.l << std::endl;
			std::cout << "==========\n";
			std::cout << "Arc a3" << std::endl;
			std::cout << "a3.x0: " << curve->a3.x0 << ", " << "a3.y0: " << curve->a3.y0 << ", " << "a3.th0: " << curve->a3.th0 << std::endl;
			std::cout << "a3.xf: " << curve->a3.xf << ", " << "a3.yf: " << curve->a3.yf << ", " << "a3.thf: " << curve->a3.thf << std::endl;
			std::cout << "a3.k: " << curve->a3.k << ", " << "a3.l: " << curve->a3.l << std::endl;

		}

		// assert(check(sc_s1, ksigns[pidx-1][0] * sc_Kmax, sc_s2, ksigns[pidx-1][1] * sc_Kmax, sc_s3, ksigns[pidx-1][2] * sc_Kmax, sc_th0, sc_thf));

	}
}

/*====================================================================
=============================Plot Functions===========================
====================================================================*/

void plotarc
(dubinsarc_out *arc, std::vector<std::vector<long double>> &points)
{
	std::vector<long double> temp;
	temp.push_back(arc->x0);
	temp.push_back(arc->x0);
	points.insert(points.begin(),temp);

	std::vector<std::vector<long double>>::iterator row;
	std::vector<long double>::iterator col;

	for 
	(int i = 1; i <= no_of_samples; i++)
	{
		long double s = arc->l / no_of_samples * i;

		long double x,y,th = 0.0;

		circline(s, arc->x0, arc->y0, arc->th0, arc->k, x, y, th);

		std::vector<long double> temp2;

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
			std::cout << "i: " << i << " x: " << points[i][0] << ", " << " y: " << points[i][1] << std::endl; 
		}
		std::cout << "======================================\n";
	}

}

void plot_dubins
(dubinscurve_out *curve, std::vector<std::vector<long double>> &c1, std::vector<std::vector<long double>> &c2, std::vector<std::vector<long double>> &c3)
{
	plotarc(&curve->a1, c1);
	plotarc(&curve->a2, c2);
	plotarc(&curve->a3, c3);

}