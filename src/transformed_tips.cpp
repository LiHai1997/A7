#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  Eigen::VectorXd tips =  Eigen::VectorXd::Zero(3*b.size());

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> T;
  forward_kinematics(skeleton, T);

  for (int i = 0; i < b.size(); i++){
    int index = b[i];
    Eigen::Vector4d tip_ori = Eigen::Vector4d(skeleton[index].length,0,0,1);
    Eigen::Vector4d tip_tra = T[index] * skeleton[index].rest_T * tip_ori;
    for(int j =0; j < 3; j++){
      tips[i*3 + j] = tip_tra[j];
    }
  }
  return tips;
  /////////////////////////////////////////////////////////////////////////////
}
