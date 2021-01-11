#include <ros/ros.h>
// #include <pcl/console/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <string.h>
#include <vector>
#include <iostream>

namespace robosense
{
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eigenVector;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> EigenAffineVector;

class LidRtkCaib
{
public:
  LidRtkCaib();
  /**打印系统依赖和本工具的版本
   */
  void version();
  void setParas(const std::string f_bag, const std::string rtk_t, const std::string imu_t, const std::string lidar_t);
  // f_lidar_pose input, lidar pose file
  void setLidarPose(const std::string f_lidar_pose, const int frame_start = 0);
  // lidar相对rtk的位姿
  Eigen::Affine3d calib();

private:
  void getRtkPose();
  void readLidarPose();
  void synTopic();

  // 标定原始数据
  std::string fname_bag_;
  // lidar 制图得到的pose文件
  std::string fname_lidar_pose_;
  std::string imu_topic_;
  std::string rtk_topic_;
  std::string lidar_topic_;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> v_rtk_pose_;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> v_lidar_pose_;
  int lidar_start_frame_;
  EigenAffineVector ev_rtk_pose_;
  EigenAffineVector ev_lidar_pose_;
};

}  // namespace robosense
