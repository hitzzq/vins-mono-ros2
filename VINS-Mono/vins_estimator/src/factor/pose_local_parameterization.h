#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class PoseManifold : public ceres::Manifold
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};
