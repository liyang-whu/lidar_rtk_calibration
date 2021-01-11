/*
 * =====================================================================================
 *
 *       Filename:  relative_pose_tool.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  02/27/2019 05:58:05 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lequeint Qu (),
 *   Organization:
 *
 * =====================================================================================
 * */
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
/**
 * 相对初始位置
 */
int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cout << argv[0] << " input.pcd "
              << " output.pcd" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointNormal> pcloud_in, pcloud_out;
  pcl::io::loadPCDFile(argv[1], pcloud_in);

  Eigen::Matrix4f tf_first_inv = Eigen::Matrix4f::Identity();
  Eigen::Affine3f af_point_b = Eigen::Affine3f::Identity();
  Eigen::Quaternionf quat_point;
  quat_point.setIdentity();
  for (std::size_t i = 0; i < pcloud_in.size(); ++i)
  {
    pcl::PointNormal& point = pcloud_in.at(i);
    af_point_b.setIdentity();
    quat_point.x() = point.normal_x;
    quat_point.y() = point.normal_y;
    quat_point.z() = point.normal_z;
    quat_point.w() = point.curvature;
    af_point_b.prerotate(quat_point);
    af_point_b.pretranslate(point.getVector3fMap());
    if (i == 0)
    {
      tf_first_inv = af_point_b.matrix().inverse();
    }
    Eigen::Affine3f af_point_a(tf_first_inv * af_point_b.matrix());
    pcl::PointNormal point_a;
    point_a.x = af_point_a.translation().x();
    point_a.y = af_point_a.translation().y();
    point_a.z = af_point_a.translation().z();
    Eigen::Quaternionf qua(af_point_a.rotation());
    point_a.normal_x = qua.x();
    point_a.normal_y = qua.y();
    point_a.normal_z = qua.z();
    point_a.curvature = qua.w();
    pcloud_out.push_back(point_a);
  }
  pcl::io::savePCDFileASCII(argv[2], pcloud_out);
  std::cout << "save to " << argv[2] << std::endl;

  return 0;
}

