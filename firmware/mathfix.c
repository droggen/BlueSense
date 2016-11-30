#include <stdint.h>
#include <math.h>
#include "mathfix.h"
/*
	File: mathfix
	
	Provides fixed point mathematic functions.
	
	Some code from:
		https://developer.mbed.org/users/sravet/code/fixedpoint/docs/tip/fixed__func_8h_source.html
		https://developer.mbed.org/users/sravet/code/fixedpoint/docs/aa2871c4c041/fixed__func_8cpp_source.html
		
	Benchmark of reciprocal square root show the following performance from slowest to fastest:
	1) 1.0/sqrt:	29uS	(reference, accurate)
	2) invSqrt3:	15us	(quake algorithm with fixed-point Newton-Raphson)
	3) fixrsqrt15:	10uS	(only without any Newton-Raphson iteration)
	
*/	
/******************************************************************************
	function: fixmul16
*******************************************************************************	
	Multiples two signed fixed-point numbers in 16.16 format and return a 16.16
	number.
	Uses internally a 64-bit multiplication to minimize risks of overflows.
*******************************************************************************/
/*inline int32_t fixmul16(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> 16);
}*/
/******************************************************************************
	function: fixmulf16
*******************************************************************************	
	Multiples two signed fixed-point numbers in 16.16 format and return a 16.16
	number.
	Uses internally a 32-bit multiplication; this is faster than fixmul16 but
	risks overflowing.
*******************************************************************************/
/*inline int32_t fixmulf16(int32_t a, int32_t b)
{
    return (a * b) >> 16;
}*/

/******************************************************************************
	function: fixrsqrt16
*******************************************************************************	
	Computes 1/sqrt(a) of the signed fixed-point number a in 16.16 format and 
	returns a 16.16 number.
	The maximum number is 32767.0. The function returns 32767.0 when a=0.0.
	
	The number of newton raphson iterations can be tweaked. Originally it is 3.
		
	
	Analysis from 2^-16 to 2^15 shows errors in % as follows:
	No Newton Raphson: 
	Average error: 0.038767
	min error: -0.001792
	max error: 0.409314

	1 Newton Raphson: 
	Average error: 0.008124
	min error: -0.171116
	max error: 0.412059

	2 Newton Raphson:
	Average error: 0.011530
	min error: -0.040427
	max error: 0.410701

	3 Newton Raphson:
	Average error: 0.011951
	min error: -0.005125
	max error: 0.410023

	4 Newton Raphson:
	Average error: 0.011979
	min error: -0.005174
	max error: 0.409679

	5 Newton Raphson:
	Average error: 0.011964
	min error: -0.005125
	max error: 0.409507

	10 Newton Raphson:
	Average error: 0.011976
	min error: -0.005174
	max error: 0.409345
	
	Error is distributed with fixed point yielding most often a larger value than floating point. This may be an issue for normalisation.

*******************************************************************************/
/*int32_t fixrsqrt16(int32_t a)
{
    int32_t x;
 
    static const uint16_t rsq_tab[] = { // domain 0.5 .. 1.0-1/16 
        0xb504, 0xaaaa, 0xa1e8, 0x9a5f, 0x93cd, 0x8e00, 0x88d6, 0x8432,
    };
 
    //int32_t i, exp;
	int8_t exp;
    if (a == 0) return 0x7fffffff;	// value=0.0 -> 1/sqrt(0)=inf -> return 32767.9999
	if (a == 0x10000) return a;		// value=1.0 -> 1/sqrt(1)=1.0 -> return 1.0
 
    exp = __builtin_clzl(a);
    
	//x = rsq_tab[(a>>(28-exp))&0x7]<<1;
	
	// Dan: The original code didn't check for exp>28. The C standard indicates it is undefined behavior to shift by a negative number.
	// On Intel the result is as expected (idx=0), whereas on AVR idx differs which changes the result slightly (lower value returned compared to expected). 
	// This test fixes this. This affects only numbers 0<=a<=7: the test could be removed if this a larger error on small input values is tolerable.	
	unsigned char idx;
	if(exp>28)
    	idx=0;					// Potentially removable
    else
    	idx = (a>>(28-exp))&0x7;
    x = ((int32_t)rsq_tab[idx])<<1;
 
    exp -= 16;
    if (exp <= 0)
        x >>= -exp>>1;
    else
        x <<= (exp>>1)+(exp&1);
 
    //if (exp&1) x = fixmul<16>(x, rsq_tab[0]);
	//if (exp&1) x = (int32_t)(((int64_t)x * (int64_t)rsq_tab[0]) >> 16);			// Check speed if calling fixmul instead
	if (exp&1) x = fixmul16(x, rsq_tab[0]);
 
	// Q16 here
 
	return x;																// Possibly return before Newton-Rapthson
 
    // newton-raphson 
    // x = x/2*(3-(a*x)*x) 
    unsigned char i = 0;
    do {
        //x = fixmul<16>((x>>1),((1<<16)*3 - fixmul<16>(fixmul<16>(a,x),x)));
		x = fixmul16((x>>1),(0x30000 - fixmul16(fixmul16(a,x),x)));
    //} while(++i < 3);
	} while(++i < 1);
 
    return x;
}

inline int32_t fixrsqrt15(int32_t a)
{
	if (a == 0) return 0x7fffffff;	// value=0.0 -> 1/sqrt(0)=inf -> return 65535.9999
	return fixrsqrt16(a<<1)>>1;
}
inline _Accum fixrsqrt15a(_Accum a)
{
	int32_t r = fixrsqrt15(*(int32_t*)&a);
	return *(_Accum*)&r;
}*/


/*int32_t fixrsqrt15b(int32_t a15)
{
	// Convert number to .16
	int32_t a = a15<<=1;
    int32_t x;
 
    static const uint16_t rsq_tab[] = { // domain 0.5 .. 1.0-1/16 
        0xb504, 0xaaaa, 0xa1e8, 0x9a5f, 0x93cd, 0x8e00, 0x88d6, 0x8432,
    };
 
    //int32_t i, exp;
	int8_t exp;
    if (a == 0) return 0x7fffffff;	// value=0.0 -> 1/sqrt(0)=inf -> return 65535.9999
	if (a == 0x10000) return a;		// value=1.0 -> 1/sqrt(1)=1.0 -> return 1.0
 
    exp = __builtin_clzl(a);
    
	//x = rsq_tab[(a>>(28-exp))&0x7]<<1;
	
	// Dan: The original code didn't check for exp>28. The C standard indicates it is undefined behavior to shift by a negative number.
	// On Intel the result is as expected (idx=0), whereas on AVR idx differs which changes the result slightly (lower value returned compared to expected). 
	// This test fixes this. This affects only numbers 0<=a<=7: the test could be removed if this a larger error on small input values is tolerable.	
	unsigned char idx;
	if(exp>28)
    	idx=0;					// Potentially removable
    else
    	idx = (a>>(28-exp))&0x7;
    x = ((int32_t)rsq_tab[idx])<<1;
 
    exp -= 16;
    if (exp <= 0)
        x >>= -exp>>1;
    else
        x <<= (exp>>1)+(exp&1);
 
    //if (exp&1) x = fixmul<16>(x, rsq_tab[0]);
	//if (exp&1) x = (int32_t)(((int64_t)x * (int64_t)rsq_tab[0]) >> 16);			// Check speed if calling fixmul instead
	if (exp&1) x = fixmul16(x, rsq_tab[0]);
 
	// Q16 here
	
	// Convert to Q15
	x>>=1; 
	//return x;																// Possibly return before Newton-Raphson
	
	_Accum xk = *(_Accum*)&x;
	_Accum ak = *(_Accum*)&a15;
	
	_Accum xk2,ak2;
	xk2=xk;
	ak2=ak;
 
    // newton-raphson 
    // x = x/2*(3-(a*x)*x) 
    unsigned char i = 0;
    do {
        //x = fixmul<16>((x>>1),((1<<16)*3 - fixmul<16>(fixmul<16>(a,x),x)));
		//x = fixmul16((x>>1),(0x30000 - fixmul16(fixmul16(a,x),x)));
		//xk = (xk/2k)*(3.0k-ak*xk*xk);
		xk2 = (xk2/2k)*(3.0k-ak2*xk2*xk2);
    } while(++i < 3);
	//} while(++i < 1);
 
	xk = xk2;
	x = *(int32_t*)&xk;
    return x;
}*/

// q is the precision of the input
// output has 32-q bits of fraction
/*inline int fixinv16(int32_t a)
{
    int32_t x;
 
    char sign = 0;
 
    if (a < 0) {
        sign = 1;
        a = -a;
    }
 
    static const uint16_t rcp_tab[] = { 
        0x8000, 0x71c7, 0x6666, 0x5d17, 0x5555, 0x4ec4, 0x4924, 0x4444
    };
        
    //int32_t exp = detail::CountLeadingZeros(a);
	int32_t exp = __builtin_clzl(a);
    x = ((int32_t)rcp_tab[(a>>(28-exp))&0x7]) << 2;
    exp -= 16;
 
    if (exp <= 0)
        x >>= -exp;
    else
        x <<= exp;
 
    // two iterations of newton-raphson  x = x(2-ax) 
    x = fixmul16(x,((0x20000) - fixmul16(a,x)));
    x = fixmul16(x,((0x20000) - fixmul16(a,x)));
 
    if (sign)
        return -x;
    else
        return x;
}*/

/*static inline int32_t fast_div16(int32_t a, int32_t b)
{
    if ((b >> 24) && (b >> 24) + 1) {
        return fixmul16(a >> 8, fixinv16(b >> 8));
    } else {
        return fixmul16(a, fixinv16(b));
    }
}
 
int32_t fixsqrt16(int32_t a) 
{
    int32_t s;
    int32_t i;
    s = (a + 0x10000) >> 1;
    // 6 iterations to converge 
    for (i = 0; i < 6; i++)
        s = (s + fast_div16(a, s)) >> 1;
    return s;
}*/


/******************************************************************************
	function: invSqrt3
*******************************************************************************	
	Reciprocal square root.
	Uses the quake algorithm with a fixed point Newton-Raphson step.
	Internally uses a float representation which pulls conversion from fixed to float and back.
	Accuracy decreases as value of x tends to zero.
*******************************************************************************/
#ifndef __cplusplus
_Accum invSqrt3(_Accum x) {
	_Accum halfx = x>>1;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	
	// One iteration of newton raphson in float
	y = *(float*)&i;
	
	_Accum yi = y;
	
	yi = yi * (1.5k - (halfx * yi * yi));
	//yi = yi * (1.5k - (halfx * yi * yi));
	return yi;
}
#endif


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root


/*int32_t _FP_SquareRoot(int32_t val, int32_t Q) {
  int32_t sval = 0;
  
  //printf("_FP_SquareRoot %ld %ld\n",val,Q);
 
  //convert Q to even
  if (Q & 0x01) {
    Q -= 1;
    val >>= 1;
  }
  //integer square root math
  for (uint8_t i=0; i<=30; i+=2) {
    if ((0x40000001>>i) + sval <= val) {  
      val -= (0x40000001>>i) + sval;     
      sval = (sval>>1) | (0x40000001>>i);
    } else {
      sval = sval>>1;
    }
  }
  if (sval < val) 
    ++sval;  
  //this is the square root in Q format
  sval <<= (Q)/2;
  //convert the square root to Q15 format
  if (Q < 15)
    return(sval<<(15 - Q));
  else
    return(sval>>(Q - 15));
}

int32_t _FP_SquareRootX(int32_t val) {
	int32_t sval = 0;
	//printf("_FP_SquareRoot %ld\n",val);

	val>>=1;

	//integer square root math
	for (uint8_t i=0; i<=30; i+=2) {
		if ((0x40000001>>i) + sval <= val) {  
		  val -= (0x40000001>>i) + sval;     
		  sval = (sval>>1) | (0x40000001>>i);
		} else {
		  sval = sval>>1;
		}
	}
	if (sval < val) 
		++sval;  
	return sval<<8;
}

_Accum sqrts15_old(_Accum v)
{
	// Get the int value
	int32_t val;
	int32_t r;
	_Accum rfp;
	
	val = *(int32_t *)&v;
	
	//r=_FP_SquareRoot(val,15);
	r=_FP_SquareRootX(val);
	rfp = *(_Accum*)&r;
	return rfp;
}*/

float invSqrtflt_ref(float x)
{
	return 1.0/sqrt(x);
}
/*
The algorithm computes 1/vx by performing the following steps:

    Alias the argument x to an integer, as a way to compute an approximation of log2(x)
    Use this approximation to compute an approximation of log2(1/vx)
    Alias back to a float, as a way to compute an approximation of the base-2 exponential
    Refine the approximation using a single iteration of the Newton's method.
	
	Y_(n+1) = y_n * (3 - x*y_n^2)/2
*/
float invSqrtflt(float x) {
	//printf("inv of %f\n",x);
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	
	// One iteration of newton raphson in float
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	//printf("is %d\n",y);
	return y;
}



/*inline uint32_t CountLeadingZeros(uint32_t x)
{
	uint32_t exp = 31;

	if (x & 0xffff0000) { 
		exp -= 16; 
		x >>= 16; 
	}

	if (x & 0xff00) { 
		exp -= 8; 
		x >>= 8; 
	}
	
	if (x & 0xf0) { 
		exp -= 4; 
		x >>= 4; 
	}

	if (x & 0xc) { 
		exp -= 2; 
		x >>= 2; 
	}
	
	if (x & 0x2) { 
		exp -= 1; 
	}

	return exp;
}
*/



/*
	1.0k/sqrts15:			2700-3700
	1.0k/sqrt:				2800-3000
	1.0k/invSqrtflt:		2790-2970
	return invSqrtflt:		2690-2900
	return x:				2390-2720
*/

// From http://www.olliw.eu/2014/fast-functions/#invsqrt
// 
/*_Accum invSqrtOlliw(_Accum x)
{
	_Accum y;
	// y1 = 3/2 - 1/2*x
	y = 1.5k - (x>>1);
	// y2 = (1.5 - x * y1* (y1/2) ) * y1
	//y = q30_mul( Q30(1.5) - q30_mul( x, q30_mul(y,y>>1) ) , y );
	y = (1.5k-x*y*(y>>1))*y;
	y = (1.5k-x*y*(y>>1))*y;
	return y;
}
*/  
#ifndef __cplusplus
_Accum invSqrt(_Accum x) {
/*	_Accum halfx = 0.5k * x;
	_Accum y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(_Accum*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;*/
	
	//return 1.0k/sqrt(x);
	//return 1.0k/sqrts15(x);
	//return 1.0f/invSqrtflt(x);
	
	//return x;
	//float r = 1.0/sqrt(x);
	//printf("invsqrt of %f is %f\n",(float)x,r);
	//return r;
	
	//return invSqrtflt(x);	// leads to errors in normalisation
	//return invSqrtflt_ref(x);
	return invSqrt3(x);		// leads to errors in normalisation
	//return fixrsqrt15a(x);
	
	//return invSqrtOlliw(x);
	
	//return x;
	
	//_FP_SquareRoot(a, 15);
	
}
_Accum invSqrt2(_Accum x) {

	//return 1.0k/sqrtF2Fa(x);
	return 0;
	
	
}
#endif
/*int32_t invSqrt4(int32_t x)
{
	_Accum r = 1.0k/sqrt(x/32768.0);
	return *(int32_t*)&r;	
}*/

/*unsigned int root(unsigned int x)
{
	unsigned int a,b;
	b = x;
	a = x = 0x3f;
	x = b/x;
	a = x = (x+a)>>1;
	x = b/x;
	a = x = (x+a)>>1;
	x = b/x;
	x = (x+a)>>1;
	return x;                      
}*/



// Performs a square root of a s16.15 number.
/*_Accum sqrts15(_Accum v)
{
	unsigned long val;		// input value
	unsigned long sval = 0;	// output value
	unsigned long r;			// tmp for casting
	_Accum rfp;			// tmp for casting
	
	val = *(unsigned long *)&v;
 
	val>>=1;

	//integer square root math
	for (uint8_t i=0; i<=30; i+=2) {
		if ((0x40000001>>i) + sval <= val) {  
		  val -= (0x40000001>>i) + sval;     
		  sval = (sval>>1) | (0x40000001>>i);
		} else {
		  sval = sval>>1;
		}
	}
	if (sval < val) 
		++sval;  
	
	sval<<=8;
	
	rfp = *(_Accum*)&sval;
	return rfp;
}*/

/*unsigned long sqrtF2F ( unsigned long X ) {
	// takes a .15 number and returns a .15
	// https://github.com/chmike/fpsqrt


	unsigned long t, q, b, r;

	// r = v - x²
	// q = 2ax
	// b = a²
	// t = 2ax + a2
	
	r = X<<1;
	b = 0x40000000;
	q = 0;

	while( b >= 256 ) {
		t = q + b;
		if( r >= t ) {
			r = r - t;
			q = t + b;
		}
		r = r * 2;     // shift left  1 bit 
		b = b / 2;     // shift right 1 bit 
	}
	//q = q / 256;     // shift right 8 bits 
	//q+=256;
	q=q/512;
	//q=q/2;

	return( q );
}*/

/*_Accum sqrtF2Fa ( _Accum _X ) {
	// takes a .15 number and returns a .15
	unsigned long X;			// tmp for casting
	_Accum rfp;			// tmp for casting
	unsigned long t, q, b, r;

	X = *(unsigned long *)&_X;

	r = X<<1;
	b = 0x40000000;
	q = 0;

	while( b >= 256 ) {
	t = q + b;
	if( r >= t ) {
	  r = r - t;
	  q = t + b;
	}
	r = r * 2;     // shift left  1 bit 
	b = b / 2;     // shift right 1 bit 
	}
	//q = q / 256;     // shift right 8 bits 
	//q+=256;
	q=q/512;
	//q=q/2;
	
	

	rfp = *(_Accum*)&q;
	//printf("[%ld %ld]\n",q,rfp);
	return rfp;
}*/

// fisqrt: http://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2/10330951
// takes an integer and returns an integer
/*unsigned long ftbl[33]={0,1,1,2,2,4,5,8,11,16,22,32,45,64,90,128,181,256,362,512,724,1024,1448,2048,2896,4096,5792,8192,11585,16384,23170,32768,46340};
unsigned long ftbl2[32]={ 32768,33276,33776,34269,34755,35235,35708,36174,36635,37090,37540,37984,38423,38858,39287,39712,40132,40548,40960,41367,41771,42170,42566,42959,43347,43733,44115,44493,44869,45241,45611,45977};
unsigned long fisqrt(unsigned long val)
{
    unsigned long cnt=0;
    unsigned long t=val;
    while (t) {cnt++;t>>=1;}
    if (6>=cnt)    t=(val<<(6-cnt));
    else           t=(val>>(cnt-6));

    return (ftbl[cnt]*ftbl2[t&31])>>15;
}*/

float fsqrt(float f)
{
	return sqrt(f);
	//return f;
}
