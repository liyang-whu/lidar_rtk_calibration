
#include "../include/calib/types.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

int main(int argc, char* argv[])
{
  Eigen::Affine3d ta = Eigen::Affine3d::Identity();
  ta.translate(Eigen::Vector3d(1, 2, 3));
  std::cout << "ta:\n" << ta.matrix() << std::endl;

  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  double roll = 0, pitch = 0, yaw = 0;
  yaw = DEG2RAD(30);
  tf.prerotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  tf.pretranslate(Eigen::Vector3d(0, 0, 1));
  std::cout << "tf:\n" << tf.matrix() << std::endl;

  Eigen::Affine3d tb = ta * tf;
  std::cout << "tb\n" << tb.matrix() << std::endl;

  return 0;
}

