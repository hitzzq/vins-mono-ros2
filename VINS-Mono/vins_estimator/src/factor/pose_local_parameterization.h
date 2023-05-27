#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class PoseLocalParameterization : public ceres::Manifold
{
    #if 1
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
    #endif
  int AmbientSize() const override { return 4; }
  int TangentSize() const override { return 3; }
  bool PlusJacobian(const double* x, double* jacobian) const override {
    return true;
  }

  bool RightMultiplyByPlusJacobian(const double* x,
                                   const int num_rows,
                                   const double* ambient_matrix,
                                   double* tangent_matrix) const override {
    return true;
  }

  bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override {
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    return true;
  }  
};
