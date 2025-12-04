#include <cmath>
#include <vector>
#include <assert.h>

/*====================================================================
==========================Data structures=============================
====================================================================*/
struct dubinsarc_out
{
	long double x0, y0, th0, k, l, xf, yf, thf;
};

struct dubinscurve_out
{
	dubinsarc_out a1;
	dubinsarc_out a2;
	dubinsarc_out a3;
	long double L;
};

struct point
{
	long double x,y,th;
};

/*====================================================================
================================Data==================================
====================================================================*/

extern bool DEBUG;

extern long double X0;
extern long double Y0;
extern long double Xf;
extern long double Yf;
extern long double Th0;
extern long double Thf;
extern long double Kmax;
extern int pidx;

extern int no_waypts;
extern int step;
extern long double angle_step;

extern int no_of_samples;

extern dubinscurve_out dubin_curve;
extern point init, final;

extern std::vector<point> best_path;

/*====================================================================
============================Helper functions==========================
====================================================================*/

long double round_up(long double, int);

long double sinc(long double);

long double mod2pi(long double);

void circline(long double, long double, long double, long double, long double, long double &, long double &, long double &);

void dubins_arc(long double, long double, long double, long double, long double, dubinsarc_out *);

void dubins_curve(long double, long double, long double, long double, long double, long double, long double, long double, long double, dubinscurve_out *);

long double rangeSymm(long double);

bool check(long double, long double, long double, long double, long double, long double, long double, long double);

/*====================================================================
=========================Function Declarations========================
====================================================================*/

void LSL(long double, long double, long double, long double &, long double &, long double &, bool &);

void RSR(long double, long double, long double, long double &, long double &, long double &, bool &);	

void LSR(long double, long double, long double, long double &, long double &, long double &, bool &);

void RSL(long double, long double, long double, long double &, long double &, long double &, bool &);
	
void RLR(long double, long double, long double, long double &, long double &, long double &, bool &);

void LRL(long double, long double, long double, long double &, long double &, long double &, bool &);

void scale_to_standard(long double, long double, long double, long double, long double, long double, long double, long double &, long double &, long double &, long double &);

void scale_from_standard(long double, long double, long double, long double, long double &, long double &, long double &);

void dubins_shortest_path(long double, long double, long double, long double, long double, long double, long double, int &, dubinscurve_out *);

/*====================================================================
=============================Plot Functions===========================
====================================================================*/

void plotarc(dubinsarc_out *, std::vector<std::vector<long double>> &points);

void plot_dubins(dubinscurve_out *, std::vector<std::vector<int>> &c1, std::vector<std::vector<int>> &c2, std::vector<std::vector<int>> &c3);