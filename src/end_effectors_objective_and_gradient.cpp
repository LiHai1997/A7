#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton skeleton_copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd xb = transformed_tips(skeleton_copy , b);
    return (xb - xb0).dot(xb - xb0);
  };
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton skeleton_copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd xb = transformed_tips(skeleton_copy , b);

    Eigen::MatrixXd jobcobian_M;
    kinematics_jacobian(skeleton_copy, b, jobcobian_M);

    Eigen::VectorXd diff_E = Eigen::VectorXd::Zero(3*b.size());

    double h = 1.0e-7;
    double E0 = f(A);

    for(int i = 0; i< diff_E.size(); i++){
      Eigen::VectorXd xb_h = xb;
      xb_h[i] += h;
      double E_h = (xb_h - xb0).dot(xb_h - xb0);
      double diff = (E_h - E0)/h;
      diff_E[i] = diff;
    }
    return jobcobian_M.transpose() * diff_E;
  };
  proj_z = [&](Eigen::VectorXd & A)
  {
    for(int i = 0; i < skeleton.size(); i++){
      for(int j = 0; j < 3; j ++) {
        A[3 * i + j] = std::max(skeleton[i].xzx_min[j], std::min(skeleton[i].xzx_max[j], A[3*i + j]));
      }

    }
    assert(skeleton.size()*3 == A.size());
  };
  /////////////////////////////////////////////////////////////////////////////
}
