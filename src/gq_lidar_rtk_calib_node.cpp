#include <ros/ros.h>
#include "gq_lidar_rtk_calib.h"
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

using namespace cicv;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_rtk_calib");
  ros::NodeHandle nh("~");
  LidRtkCaib lid_rtk_calib;

  std::string calib_bag_file;
  nh.getParam(std::string("calib_bag_file"), calib_bag_file);
  std::cout << "calibation bag file : " << calib_bag_file << std::endl;
  rosbag::Bag bag;
  bag.open(calib_bag_file, rosbag::bagmode::Read);

  std::string rtk_topic;
  nh.getParam(std::string("rtk_topic"), rtk_topic);

  std::cout << "rtk_topic : " << rtk_topic << std::endl;
  std::string imu_topic;
  nh.getParam(std::string("imu_topic"), imu_topic);

  std::cout << "imu_topic: " << imu_topic << std::endl;

  std::string lidar_topic;
  nh.getParam(std::string("lidar_topic"), lidar_topic);
  std::cout << "lidar_topic : " << lidar_topic << std::endl;

  std::string lidar_pose_file;
  nh.getParam(std::string("lidar_pose_file"), lidar_pose_file);
  std::cout << "lidar_pose_file : " << lidar_pose_file << std::endl;

  lid_rtk_calib.setParas(calib_bag_file, rtk_topic, imu_topic, lidar_topic);
  lid_rtk_calib.setLidarPose(lidar_pose_file, 20);

  Eigen::Affine3d trans;
  // trans.setIdentity();
  trans = lid_rtk_calib.calib();

  std::string save_file;
  nh.getParam(std::string("save_file"), save_file);
  std::cout << "save_file : " << save_file << std::endl;

  cv::FileStorage fs(save_file, cv::FileStorage::WRITE);
  if (fs.isOpened())
  {
    cv::Mat_<double> t1cv = cv::Mat_<double>::ones(4, 4);
    cv::eigen2cv(trans.matrix(), t1cv);
    fs << "trans" << t1cv;
    fs.release();
  }
  else
  {
    std::cerr << "failed to open output file " << save_file << "\n";
    return 1;
  }

  return 0;
}
