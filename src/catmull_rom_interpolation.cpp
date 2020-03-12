#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  if (keyframes.size() == 0){
    return Eigen::Vector3d(0,0,0);
  }

  t = std::fmod(t, keyframes.back().first);

  int index = keyframes.size();
  for (int i = 0; i < keyframes.size(); i++){
    if (keyframes[i].first > t){
      index = index;
      break;
    }
  }

  Eigen::Vector3d p1, p2, p3, p4;
  double t1, t2, t3, t4;
  int size = keyframes.size();

  if (index < 2 || size <= index + 1){
    p3 = keyframes[index].second;
    t3 = keyframes[index].first;
    if (index == 0) {
      p1 = keyframes[size -2].second;
      p2 = keyframes[size -1].second;
      p4 = keyframes[index+1].second;

      t1 = keyframes[size -2].first;
      t2 = keyframes[size -1].first;
      t4 = keyframes[index+1].first;

    } else if (index == 1){
      p1 = keyframes[size - 1].second;
      p2 = keyframes[index - 1].second;
      p4 = keyframes[index + 1].second;

      t1 = keyframes[size - 1].first;
      t2 = keyframes[index - 1].first;
      t4 = keyframes[index + 1].first;
    }else{
      p1 = keyframes[index - 2].second;
      p2 = keyframes[index - 1].second;
      p4 = keyframes[0].second;

      t1 = keyframes[index - 2].first;
      t2 = keyframes[index - 1].first;
      t4 = keyframes[0].first;
    }
  } else{
    p1 = keyframes[index - 2].second;
    p2 = keyframes[index - 1].second;
    p3 = keyframes[index].second;
    p4 = keyframes[index + 1].second;

    t1 = keyframes[index - 2].first;
    t2 = keyframes[index - 1].first;
    t3 = keyframes[index].first;
    t4 = keyframes[index + 1].first;
  }

  Eigen::Vector3d A1 = (t2 - t)/(t2 - t1) * p1 + (t - t1)/(t2 - t1) * p2;
  Eigen::Vector3d A2 = (t3 - t)/(t3 - t2) * p2 + (t - t2)/(t3 - t2) * p3;
  Eigen::Vector3d A3 = (t4 - t)/(t4 - t3) * p3 + (t - t3)/(t4 - t3) * p4;
  Eigen::Vector3d B1 = (t3 - t)/(t3 - t1) * A1 + (t - t1)/(t3 - t1) * A2;
  Eigen::Vector3d B2 = (t4 - t)/(t4 - t2) * A2 + (t - t2)/(t4 - t2) * A3;
  return (t3 - t)/(t3 - t2) * B1 + (t - t2)/(t3 - t2) * B2;

  /////////////////////////////////////////////////////////////////////////////
}
