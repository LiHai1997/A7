#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  double a = max_step; 
  double E = f(z);

  Eigen::VectorXd z_m = z - a * dz;
  proj_z(z_m);
  while (f(z_m) > E){
    a *= 0.5;
    z_m = z - a * dz;
    proj_z(z_m);
  }
  return a;
  /////////////////////////////////////////////////////////////////////////////
}
