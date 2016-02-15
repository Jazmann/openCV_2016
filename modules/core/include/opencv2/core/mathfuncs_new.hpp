//
//  mathfuncs_new.hpp
//  OpenCV
//
//  Created by Jasper Shemilt on 25/01/2016.
//
//

#ifndef mathfuncs_new_h
#define mathfuncs_new_h

#ifndef __cplusplus
#  error core.hpp header must be compiled as C++
#endif

#include <cmath>

///////////////////////////// Bitwise and discrete math operations ///////////////////////////

template<typename _Tp> _Tp gComDivisor(_Tp u, _Tp v) {
    if (v)
        return gComDivisor<_Tp>(v, u % v);
    else
        return u < 0 ? -u : u; /* abs(u) */
};

template<typename _Tp> _Tp gComDivisor(_Tp a, _Tp b, _Tp c){
    return gComDivisor<_Tp>(gComDivisor<_Tp>(a, b), c);
};


template<typename _Tp> _Tp gComDivisor(_Tp a, _Tp* b, CV_32U_TYPE size_b){
    if (size_b >= 2){
        gComDivisor<_Tp>(a, b[0]);
        return gComDivisor<_Tp>(gComDivisor<_Tp>(a, b[0]), b++, size_b-1);
    }
    else if(size_b == 1) {
        return gComDivisor<_Tp>(a, b[0]);
    }
    else {
        return a;
    }
};

template<typename _Tp> _Tp gComDivisor(_Tp* b, CV_32U_TYPE size_b){
    switch (size_b) {
        case 0:
            return _Tp();
            break;
        case 1:
            return b[0];
            break;
        case 2:
            return gComDivisor<_Tp>(b[0],b[1]);
            break;
        case 3:
            return gComDivisor<_Tp>(gComDivisor<_Tp>(b[0],b[1]),b[2]);
            break;
        case 4:
            return gComDivisor<_Tp>(gComDivisor<_Tp>(b[0],b[1]), gComDivisor<_Tp>(b[2],b[3]));
            break;
        default:
            return gComDivisor<_Tp>(gComDivisor<_Tp>(b,size_b/2), gComDivisor<_Tp>(b+(size_b)/2,(size_b+1)/2));
            break;
    }
};

CV_32U_TYPE CV_INLINE mostSignificantBit(CV_64U_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0xFFFFFFFF00000000) { r += 32/1; x >>= 32/1; }
if (x & 0x00000000FFFF0000) { r += 32/2; x >>= 32/2; }
if (x & 0x000000000000FF00) { r += 32/4; x >>= 32/4; }
if (x & 0x00000000000000F0) { r += 32/8; x >>= 32/8; }
return r + bval[x];
}
CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_32U_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0xFFFF0000) { r += 16/1; x >>= 16/1; }
if (x & 0x0000FF00) { r += 16/2; x >>= 16/2; }
if (x & 0x000000F0) { r += 16/4; x >>= 16/4; }
return r + bval[x];
}

CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_16U_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0xFF00) { r += 8/1; x >>= 8/1; }
if (x & 0x00F0) { r += 8/2; x >>= 8/2; }
return r + bval[x];
}

CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_8U_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0xF0) { r += 4/1; x >>= 4/1; }
return r + bval[x];
}

CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_64S_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0x7FFFFFFF00000000) { r += 32/1; x >>= 32/1; }
if (x & 0x00000000FFFF0000) { r += 32/2; x >>= 32/2; }
if (x & 0x000000000000FF00) { r += 32/4; x >>= 32/4; }
if (x & 0x00000000000000F0) { r += 32/8; x >>= 32/8; }
return r + bval[x];
}
CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_32S_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0x7FFF0000) { r += 16/1; x >>= 16/1; }
if (x & 0x0000FF00) { r += 16/2; x >>= 16/2; }
if (x & 0x000000F0) { r += 16/4; x >>= 16/4; }
return r + bval[x];
}

CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_16S_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0x7F00) { r += 8/1; x >>= 8/1; }
if (x & 0x00F0) { r += 8/2; x >>= 8/2; }
return r + bval[x];
}

CV_32U_TYPE CV_INLINE  mostSignificantBit(CV_8S_TYPE x)
{
static const CV_32U_TYPE bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
CV_32U_TYPE r = 0;
if (x & 0x70) { r += 4/1; x >>= 4/1; }
return r + bval[x];
}


/* f : number to convert.
 * num, denom: returned parts of the rational.
 * max_denom: max denominator value.  Note that machine floating point number
 *     has a finite resolution (10e-16 ish for 64 bit double), so specifying
 *     a "best match with minimal error" is often wrong, because one can
 *     always just retrieve the significand and return that divided by
 *     2**52, which is in a sense accurate, but generally not very useful:
 *     1.0/7.0 would be "2573485501354569/18014398509481984", for example.
 */
void CV_INLINE rat_approx(double f, CV_64S_TYPE max_denom, CV_64S_TYPE *num, CV_64S_TYPE *denom)
{
/*  a: continued fraction coefficients. */
CV_64S_TYPE a, h[3] = { 0, 1, 0 }, k[3] = { 1, 0, 0 };
CV_64S_TYPE x, d, n = 1;
int i, neg = 0;

if (max_denom <= 1) { *denom = 1; *num = (CV_64S_TYPE) f; return; }

if (f < 0) { neg = 1; f = -f; }

while (f != floor(f)) { n <<= 1; f *= 2; }
d = f;

/* continued fraction and check denominator each step */
for (i = 0; i < 64; i++) {
    a = n ? d / n : 0;
    if (i && !a) break;
    
    x = d; d = n; n = x % n;
    
    x = a;
    if (k[1] * a + k[0] >= max_denom) {
        x = (max_denom - k[0]) / k[1];
        if (x * 2 >= a || k[1] >= max_denom)
            i = 65;
        else
            break;
    }
    
    h[2] = x * h[1] + h[0]; h[0] = h[1]; h[1] = h[2];
    k[2] = x * k[1] + k[0]; k[0] = k[1]; k[1] = k[2];
}
*denom = k[1];
*num = neg ? -h[1] : h[1];
}

// Error function methods

 double CV_EXPORTS erf(double x);
 double CV_EXPORTS erf(double a, double b);
 double CV_EXPORTS erfinv(double x); // returns  the inverse error function

#endif /* mathfuncs_new_h */
