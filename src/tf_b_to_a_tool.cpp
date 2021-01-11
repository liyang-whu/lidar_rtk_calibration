
#include "../include/calib/types.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
/**
 * tf： frame B相对 frame A的变换矩阵， tf*a =b
 *  读入opecv yaml格式的tf
 */
int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cout << argv[0] << " input_b.pcd "
              << " output_a_est.pcd"
              << " tranf.yaml " << std::endl;
    return -1;
  }
  std::ifstream ifs;
  std::string line, temp;
  ifs.open(argv[1]);
  std::cout << "open file " << argv[1] << std::endl;
  pcl::PointCloud<pcl::PointNormal> pcloud_in;
  pcl::io::loadPCDFile(argv[1], pcloud_in);

  cv::FileStorage fs2(argv[3], cv::FileStorage::READ);
  cv::Mat cv_tf;
  fs2["trans"] >> cv_tf;
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  cv::cv2eigen(cv_tf, tf.matrix());
  std::cout << "tf\n" << tf.matrix() << std::endl;
  float roll = 0, pitch = 0, yaw = 0;
  pcl::getEulerAngles(tf, roll, pitch, yaw);
  std::cout << "yaw = " << yaw * 57.3 << std::endl;
  tf.prerotate(Eigen::AngleAxisf(-0.0 / 57, Eigen::Vector3f::UnitZ()));
  pcl::getEulerAngles(tf, roll, pitch, yaw);
  std::cout << "yaw = " << yaw * 57.3 << std::endl;

  Eigen::Matrix4f tf_inv = tf.matrix().inverse();

  Eigen::Affine3f af_point_b = Eigen::Affine3f::Identity();

  pcl::PointCloud<pcl::PointNormal> pcloud_out;
  for (std::size_t i = 0; i < pcloud_in.size(); ++i)
  {
    pcl::PointNormal& point = pcloud_in.at(i);
    af_point_b.setIdentity();
    Eigen::Quaternionf quat_point;
    quat_point.x() = point.normal_x;
    quat_point.y() = point.normal_y;
    quat_point.z() = point.normal_z;
    quat_point.w() = point.curvature;
    af_point_b.prerotate(quat_point);
    af_point_b.pretranslate(point.getVector3fMap());
    Eigen::Affine3f af_point_a_est(tf * af_point_b.matrix() * tf_inv);  //由body frame B计算得到的body frame A的坐标原点
    // Eigen::Affine3f af_point_a_est(tf * af_point_b.matrix() );  //B在A坐标系下
    pcl::PointNormal point_a_est;
    point_a_est.x = af_point_a_est.translation().x();
    point_a_est.y = af_point_a_est.translation().y();
    point_a_est.z = af_point_a_est.translation().z();
    Eigen::Quaternionf qua(af_point_a_est.rotation());
    point_a_est.normal_x = qua.x();
    point_a_est.normal_y = qua.y();
    point_a_est.normal_z = qua.z();
    point_a_est.curvature = qua.w();
    pcloud_out.push_back(point_a_est);
  }
  pcl::io::savePCDFileASCII(argv[2], pcloud_out);
  std::cout << "save to " << argv[2] << std::endl;

  return 0;
}

