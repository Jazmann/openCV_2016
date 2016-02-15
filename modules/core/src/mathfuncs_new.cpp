//
//  mathfuncs_new.hpp
//  OpenCV
//
//  Created by Jasper Shemilt on 25/01/2016.
//
//


#include <cmath>

namespace cv {
    
double erf(double x);
double erf(double a, double b);
double erfinv(double x);
    
    
double erf(double x)
{
int sign;
// constants
double a1 =  0.254829592;
double a2 = -0.284496736;
double a3 =  1.421413741;
double a4 = -1.453152027;
double a5 =  1.061405429;
double p  =  0.3275911;

// Save the sign of x
x < 0 ? sign = -1 : sign = 1;
    x = std::fabs(x);

// A&S formula 7.1.26
double t = 1.0/(1.0 + p*x);
double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

return sign*y;
}

double erf(double a, double b)
{
// constants
const double a1 =  0.254829592;
const double a2 = -0.284496736;
const double a3 =  1.421413741;
const double a4 = -1.453152027;
const double a5 =  1.061405429;
const double p  =  0.3275911;

// Save the sign of x
int sign = 1;
if (a < 0) sign *= -1; a = fabs(a);
if (b < 0) sign *= -1; b = fabs(b);

// A&S formula 7.1.26
double pr = 1/p;
double t = (pr*b)/(a + pr*b);
double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-(a*a)/(b*b));

return sign*y;
}

double erfinv(double x)
{
// returns  the inverse error function
// x must be  <-1<x<1

int kMaxit    = 50;
double kEps   = 1e-8;
double kConst = 0.8862269254527579;     // sqrt(pi)/2.0

if(fabs(x) <= kEps) return kConst*x;

// Newton iterations
double erfi, derfi, y0,y1,dy0,dy1;
if(fabs(x) < 1.0) {
    erfi  = kConst*fabs(x);
    y0    = erf(0.9*erfi);
    derfi = 0.1*erfi;
    for (int iter=0; iter<kMaxit; iter++) {
        y1  = 1. - erfc(erfi);
        dy1 = fabs(x) - y1;
        if (std::fabs(dy1) < kEps)  {if (x < 0) return -erfi; else return erfi;}
        dy0    = y1 - y0;
        derfi *= dy1/dy0;
        y0     = y1;
        erfi  += derfi;
        if(std::fabs(derfi/erfi) < kEps) {if (x < 0) return -erfi; else return erfi;}
    }
}
return 0; //did not converge
}

}
