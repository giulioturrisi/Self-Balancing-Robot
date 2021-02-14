//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 01-Feb-2021 19:24:22
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include Files
#include "main.h"
#include "forward_dynamic_fun.h"
#include "forward_dynamic_fun_terminate.h"

// Function Declarations
static void argInit_6x1_real_T(double result[6]);
static double argInit_real_T();
static void main_forward_dynamic_fun();

// Function Definitions
//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_6x1_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 6; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_forward_dynamic_fun()
{
  double dv[6];
  double phi_ddot;
  double theta_ddot;
  double u_l_tmp;
  double x_ddot;

  // Initialize function 'forward_dynamic_fun' input arguments.
  u_l_tmp = argInit_real_T();

  // Initialize function input argument 'state'.
  // Call the entry-point 'forward_dynamic_fun'.
  argInit_6x1_real_T(dv);
  forward_dynamic_fun(u_l_tmp, u_l_tmp, dv, &theta_ddot, &phi_ddot, &x_ddot);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_forward_dynamic_fun();

  // Terminate the application.
  // You do not need to do this more than one time.
  forward_dynamic_fun_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
