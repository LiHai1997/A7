#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

Eigen::Affine3d pose_thransf(
  const Skeleton & skeleton,
  int i
){
  if (skeleton[i].parent_index == -1){
    return Eigen::Affine3d::Identity();
  }else{
    Eigen::Affine3d parent_pose_thransf = pose_thransf(skeleton, skeleton[i].parent_index);
    Eigen::Affine3d rest_T = skeleton[i].rest_T;
    Eigen::Affine3d rot = euler_angles_to_transform(skeleton[i].xzx);
    return parent_pose_thransf * rest_T * rot * rest_T.inverse();
  }
}
void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  for (int i = 0; i < skeleton.size(); i++){
    T[i] = pose_thransf(skeleton, i);
  }
  /////////////////////////////////////////////////////////////////////////////
}
