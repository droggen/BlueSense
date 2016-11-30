#ifndef __MATHFIX_H
#define __MATHFIX_H

int32_t fixmul16(int32_t a, int32_t b);
int32_t fixmulf16(int32_t a, int32_t b);
int32_t fixrsqrt16(int32_t a);
int32_t fixrsqrt15(int32_t a);
#ifndef __cplusplus
_Accum fixrsqrt15a(_Accum a);
#endif
int32_t fixrsqrt15b(int32_t a);
int32_t fixsqrt16(int32_t a);
#ifndef __cplusplus
_Accum invSqrt3(_Accum x);
#endif
int32_t _FP_SquareRoot(int32_t val, int32_t Q);
int32_t _FP_SquareRootX(int32_t val);
#ifndef __cplusplus
_Accum sqrts15(_Accum v);
_Accum invSqrtOlliw(_Accum x);
#endif
unsigned int root(unsigned int x);
unsigned long sqrtF2F ( unsigned long X );
#ifndef __cplusplus
_Accum sqrtF2Fa ( _Accum _X );
#endif
unsigned long fisqrt(unsigned long val);
float fsqrt(float f);
#ifndef __cplusplus
_Accum invSqrt(_Accum x);
_Accum invSqrt2(_Accum x);
#endif
int32_t invSqrt4(int32_t x);
float invSqrtflt(float x);
float invSqrtflt_ref(float x);
//_Accum fixrsqrt15(_Accum _a);
//uint32_t fixrsqrt15(uint32_t a);
//int32_t fixrsqrt16(int32_t a);

#endif