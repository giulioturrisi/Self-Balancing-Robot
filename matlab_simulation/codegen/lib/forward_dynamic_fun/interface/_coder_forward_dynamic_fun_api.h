/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_forward_dynamic_fun_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 01-Feb-2021 19:24:22
 */

#ifndef _CODER_FORWARD_DYNAMIC_FUN_API_H
#define _CODER_FORWARD_DYNAMIC_FUN_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void forward_dynamic_fun(real_T u_l, real_T u_r, real_T state[6], real_T
    *theta_ddot, real_T *phi_ddot, real_T *x_ddot);
  void forward_dynamic_fun_api(const mxArray * const prhs[3], int32_T nlhs,
    const mxArray *plhs[3]);
  void forward_dynamic_fun_atexit(void);
  void forward_dynamic_fun_initialize(void);
  void forward_dynamic_fun_terminate(void);
  void forward_dynamic_fun_xil_shutdown(void);
  void forward_dynamic_fun_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_forward_dynamic_fun_api.h
 *
 * [EOF]
 */
