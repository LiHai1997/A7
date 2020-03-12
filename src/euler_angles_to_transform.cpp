#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double th1 = xzx[0]/180.0 * M_PI;
  double th2 = xzx[1]/180.0 * M_PI;
  double th3 = xzx[2]/180.0 * M_PI;
  
  Eigen::Affine3d A, B, C;
  A.matrix() << 
    1, 0,        0,         0,
    0, cos(th1), -sin(th1), 0,
    0, sin(th1), cos(th1),  0,
    0, 0,        0,         1;
  
  B.matrix() <<
    cos(th2), -sin(th2), 0, 0,
    sin(th2), cos(th2),  0, 0,
    0,        0,         1, 0,
    0,        0,         0, 1;

  C.matrix() <<
    1, 0,        0,         0,
    0, cos(th3), -sin(th3), 0,
    0, sin(th3), cos(th3),  0,
    0, 0,        0,         1;

  return C*B*A;
  /////////////////////////////////////////////////////////////////////////////
}
