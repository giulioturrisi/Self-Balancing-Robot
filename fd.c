/* This file was automatically generated by CasADi 3.6.5.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) fd_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};

/* fd:(i0[6],i1[2])->(o0[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=6.1670000000000003e-02;
  a4=4.1819999999999996e-02;
  a5=-2.2497000000000010e-02;
  a6=arg[0]? arg[0][1] : 0;
  a7=sin(a6);
  a5=(a5*a7);
  a7=sin(a6);
  a5=(a5*a7);
  a4=(a4-a5);
  a3=(a3*a4);
  a5=1.5417500000000001e-01;
  a7=2.0000000000000001e-01;
  a8=cos(a6);
  a8=(a7*a8);
  a9=casadi_sq(a8);
  a9=(a5-a9);
  a9=(a4*a9);
  a3=(a3/a9);
  a10=10.;
  a11=arg[1]? arg[1][0] : 0;
  a12=(a10*a11);
  a13=arg[1]? arg[1][1] : 0;
  a10=(a10*a13);
  a12=(a12+a10);
  a10=-2.0000000000000001e-01;
  a14=(a10*a1);
  a15=sin(a6);
  a14=(a14*a15);
  a14=(a14*a1);
  a10=(a10*a2);
  a15=sin(a6);
  a10=(a10*a15);
  a10=(a10*a2);
  a14=(a14+a10);
  a10=-1.9999999999999996e-01;
  a10=(a10*a0);
  a15=2.0000000000000000e-02;
  a16=(a15*a1);
  a10=(a10+a16);
  a14=(a14+a10);
  a12=(a12-a14);
  a3=(a3*a12);
  a14=(a8*a4);
  a14=(a14/a9);
  a10=-5.7503000000000005e-02;
  a10=(a10*a2);
  a16=sin(a6);
  a10=(a10*a16);
  a16=cos(a6);
  a10=(a10*a16);
  a10=(a10*a2);
  a16=-1.9600000000000002e+00;
  a17=sin(a6);
  a16=(a16*a17);
  a10=(a10+a16);
  a15=(a15*a0);
  a16=-2.0000000000000000e-03;
  a17=(a16*a1);
  a15=(a15+a17);
  a10=(a10+a15);
  a15=(a11+a13);
  a10=(a10+a15);
  a14=(a14*a10);
  a3=(a3+a14);
  if (res[0]!=0) res[0][3]=a3;
  a3=(a8*a4);
  a3=(a3/a9);
  a3=(a3*a12);
  a12=2.5000000000000000e+00;
  a12=(a12*a4);
  a12=(a12/a9);
  a12=(a12*a10);
  a3=(a3+a12);
  a3=(-a3);
  if (res[0]!=0) res[0][4]=a3;
  a8=casadi_sq(a8);
  a5=(a5-a8);
  a5=(a5/a9);
  a13=(a13-a11);
  a7=(a7*a2);
  a11=sin(a6);
  a7=(a7*a11);
  a7=(a7*a0);
  a0=5.7503000000000005e-02;
  a11=(a0*a2);
  a9=sin(a6);
  a11=(a11*a9);
  a9=cos(a6);
  a11=(a11*a9);
  a11=(a11*a1);
  a7=(a7+a11);
  a0=(a0*a1);
  a1=sin(a6);
  a0=(a0*a1);
  a6=cos(a6);
  a0=(a0*a6);
  a0=(a0*a2);
  a7=(a7+a0);
  a16=(a16*a2);
  a7=(a7+a16);
  a13=(a13-a7);
  a5=(a5*a13);
  if (res[0]!=0) res[0][5]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int fd(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int fd_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int fd_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fd_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int fd_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fd_release(int mem) {
}

CASADI_SYMBOL_EXPORT void fd_incref(void) {
}

CASADI_SYMBOL_EXPORT void fd_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int fd_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int fd_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real fd_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fd_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fd_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fd_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fd_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int fd_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int fd_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
