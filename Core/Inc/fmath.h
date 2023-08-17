/*
 *
 *
 * FAST MATH LIBRARY
 * Do not try and comprehend it, just use it!
 *
 * Collector & Author: Tomislav Ivanis
 *
 */


#ifndef __FMATH_H
#define __FMATH_H

#include <math.h>

#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)

// evaluates polynomial at a certain point x
// a = polynomial coefficient array with size deg+1. Here a[0] is a0, a[1] is a1 etc.
// deg = polynomial degree
static inline float fmath_polyval(float *a, float x, int deg)
{
    float xpow[deg+1];
    float y=a[0];
    int i;
    if(deg>=1)
    {
        xpow[0]=1.0f;
        xpow[1]=x;
        //Multiply the exponents
        for(i=2; i<=deg; i++)
        {
            xpow[i]=xpow[i-1]*x;
        }
        //Multiply and sum the polynomial
        for(i=1; i<=deg; i++)
        {
            y=y+xpow[i]*a[i];
        }
    }
    return y;
}

static inline float fmath_atan2(float y, float x)
{
	float abs_y = fabsf(y) + 1e-20; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (3.14159265359 / 4.0);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * 3.14159265359 / 4.0);
	}

	UTILS_NAN_ZERO(angle);

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}


//https://gist.github.com/LingDong-/7e4c4cae5cbbc44400a05fba65f06f23
static inline float fmath_ln(float x)
{
  unsigned int bx = * (unsigned int *) (&x);
  unsigned int ex = bx >> 23;
  signed int t = (signed int)ex-(signed int)127;
  unsigned int s = (t < 0) ? (-t) : t;
  bx = 1065353216 | (bx & 8388607);
  x = * (float *) (&bx);
  return -1.49278+(2.11263+(-0.729104+0.10969*x)*x)*x+0.6931471806*t;
}

#endif
