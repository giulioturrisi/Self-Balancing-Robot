//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: forward_dynamic_fun.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 01-Feb-2021 19:24:22
//

// Include Files
#include "forward_dynamic_fun.h"
#include <cmath>

// Function Definitions
//
// UNTITLED Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : double u_l
//                double u_r
//                const double state[6]
//                double *theta_ddot
//                double *phi_ddot
//                double *x_ddot
// Return Type  : void
//
void forward_dynamic_fun(double u_l, double u_r, const double state[6], double
  *theta_ddot, double *phi_ddot, double *x_ddot)
{
  double b_theta_ddot_tmp;
  double theta_ddot_tmp;
  double theta_ddot_tmp_tmp;

  // kg
  // mm
  // kg m^2
  // mm
  theta_ddot_tmp_tmp = std::cos(state[2]);
  theta_ddot_tmp = u_l + u_r;
  b_theta_ddot_tmp = 11.187452640000002 * std::sin(state[2]);
  *theta_ddot = (1.826971 * b_theta_ddot_tmp - 1.1415768 * theta_ddot_tmp_tmp *
                 theta_ddot_tmp / 0.1) / (0.006107564053 - 1.30319759029824 *
    theta_ddot_tmp_tmp * theta_ddot_tmp_tmp);
  *phi_ddot = 0.35 * ((u_l - u_r) / 0.1) / 1.228343;
  *x_ddot = (theta_ddot_tmp / 0.1 - 1.1415768 * std::cos(state[2]) *
             b_theta_ddot_tmp / 0.005928) / (1.826971 - 1.30319759029824 *
    (theta_ddot_tmp_tmp * theta_ddot_tmp_tmp) / 0.005928);
}

//
// File trailer for forward_dynamic_fun.cpp
//
// [EOF]
//
