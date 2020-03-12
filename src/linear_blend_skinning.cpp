#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  U.resize(V.rows(), 3);
  for(int i=0; i < V.rows(); i++){
    Eigen::Vector4d v = Eigen::Vector4d(V(i, 0), V(i, 1), V(i, 2), 1);
    Eigen::Vector4d u = Eigen::Vector4d::Zero();
    for(int j=0; j < skeleton.size(); j++){
      if(skeleton[j].weight_index != -1){
        u += W(i, skeleton[j].weight_index) * (T[j] * v);
      }
    }
    for (int k =0; k < 3; k++){
      U(i, k) = u(k)/u(3);
    }
  }
  /////////////////////////////////////////////////////////////////////////////
}
