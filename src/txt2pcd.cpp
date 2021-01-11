//
// Created by robosense on 3/24/18.
//

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <stdio.h>

#include <fstream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/angles.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <fstream>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cout << argv[0] << " input.txt "
              << " output.pcd" << std::endl;
    return -1;
  }
  std::ifstream ifs;
  std::string line, temp;
  ifs.open(argv[1]);
  std::cout << "open file " << argv[1] << std::endl;
  pcl::PointCloud<pcl::PointNormal> pcloud;
  pcl::PointNormal point;
  while (getline(ifs, line))
  {
    float roll, pitch, yaw;
    std::stringstream sstream(line);
    sstream >> temp >> point.x >> point.y >> point.z >> roll >> pitch >> yaw;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.prerotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    trans.prerotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    trans.prerotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf qua(trans.rotation());
    // point.x /= 2.0;
    // point.y /= 2.0;
    // point.z /= 2.0;
    point.normal_x = (qua.x());
    point.normal_y = (qua.y());
    point.normal_z = (qua.z());
    point.curvature = (qua.w());
    pcloud.push_back(point);
  }
  ifs.close();
  pcl::io::savePCDFileASCII(argv[2], pcloud);
  std::cout << "save to  " << argv[2] << std::endl;
  return 0;
}

