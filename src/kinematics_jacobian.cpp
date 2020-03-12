#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  Eigen::VectorXd tip = transformed_tips(skeleton, b);
  double h = 1.0e-7;
  
  for (int i=0; i<skeleton.size(); i++){
  	// for each bone
		for (int j=0; j<3; j++){
			// for each x, z, x
			Skeleton skeleton_h = skeleton;
			skeleton_h[i].xzx[j] += h;
			Eigen::VectorXd tip_h = transformed_tips(skeleton_h, b);
			// in each column, every three entries are the derivative for dxi / dj for different bones
			J.col(3*i+j) = (tip_h - tip) / h;	
		}  
  }
  /////////////////////////////////////////////////////////////////////////////
}
