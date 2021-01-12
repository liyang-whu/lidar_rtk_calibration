/*
 * =====================================================================================
 *
 *       Filename:  extract_rtk_pose_node.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  02/12/2019 08:45:20 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lequeint Qu (),
 *   Organization:
 *
 * =====================================================================================
 */

#include <ros/ros.h>
// #include <pcl/console/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geodetic_conv.hpp"

#define foreach BOOST_FOREACH
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtk_pose_node");
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  geodetic_converter::GeodeticConverter g_geodetic_converter;
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

  std::vector<double> rtk_init(3, 0);
  bool use_user_init = false;

  if (nh.getParam(std::string("rtk_init_position"), rtk_init))
  {
    use_user_init = true;
    std::cout << "use_user_init " << rtk_init[0] << ", " << rtk_init[1] << ", " << rtk_init[2] << std::endl;
  }

  std::string rtk_pose_file;
  nh.getParam(std::string("rtk_pose_file"), rtk_pose_file);
  std::cout << "rtk_pose_file: " << rtk_pose_file << std::endl;

  bool rtk_mapping_flag = true;

  std::string lidar_topic = "/rslidar_points";
  std::vector<float> tranl(3, 0);
  std::vector<float> quat_v(4, 0);  // w,x,y,z
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  if (rtk_mapping_flag)
  {
    nh.getParam(std::string("lidar_topic"), lidar_topic);
    std::cout << "lidar_topic: " << lidar_topic << std::endl;
    nh.getParam("trans", tranl);
    nh.getParam("quat", quat_v);
    tf.prerotate(Eigen::Quaternionf(quat_v[0], quat_v[1], quat_v[2], quat_v[3]));
    tf.pretranslate(Eigen::Vector3f(tranl[0], tranl[1], tranl[2]));
    std::cout << "tf\n" << tf.matrix() << std::endl;
  }

  std::vector<std::string> topics;
  topics.push_back(imu_topic);
  topics.push_back(rtk_topic);
  if (rtk_mapping_flag)
  {
    topics.push_back(lidar_topic);
  }
  Eigen::Vector3d acc_0, gyr_0;
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double imu_t_last_ = 0.0, rtk_t_last_ = 0.0;
  int size_imu = 0, size_rtk = 0;
  pcl::PointCloud<pcl::PointNormal>::Ptr rtk_pose(new pcl::PointCloud<pcl::PointNormal>);

  pcl::PointNormal p;  // rtk pose
  int lidar_msg_num = 0;
  int map_step = 30;
  Eigen::Affine3f aff_rtk = Eigen::Affine3f::Identity();
  Eigen::Affine3f tf_first_inv;
  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
    if (imu_msg != NULL)
    {
      double t_now = imu_msg->header.stamp.sec;

      // cout << t_now << " " << imu_t_last_ << "\t" << t_now - imu_t_last_ << std::endl;
      // ROS_ASSERT(t_now - imu_t_last_ >= 1e-3);
      imu_t_last_ = t_now;
      size_imu++;
      p.normal_x = imu_msg->orientation.x;
      p.normal_y = imu_msg->orientation.y;
      p.normal_z = imu_msg->orientation.z;
      p.curvature = imu_msg->orientation.w;
    }
    sensor_msgs::NavSatFixConstPtr rtk_msg = m.instantiate<sensor_msgs::NavSatFix>();
    if (rtk_msg != NULL)
    {
      double t_now = rtk_msg->header.stamp.sec;
      // cout << t_now << " " << rtk_t_last_ << "\t" << t_now - rtk_t_last_ << std::endl;
      // ROS_ASSERT(t_now - rtk_t_last_ >= 0);
      rtk_t_last_ = t_now;
      // ROS_ASSERT(fabs(rtk_t_last_ - lidar_t_last_) < 0.02);
      Eigen::Vector3d rtk_v;
      rtk_v << rtk_msg->latitude, rtk_msg->longitude, rtk_msg->altitude;
      if (size_rtk == 0)
      {
        if (use_user_init)
        {
          g_geodetic_converter.initialiseReference(rtk_init[0], rtk_init[1], rtk_init[2]);
        }
        else
        {
          g_geodetic_converter.initialiseReference(rtk_v[0], rtk_v[1], rtk_v[2]);
        }
      }
      size_rtk++;
      double x = 0, y = 0, z = 0;
      g_geodetic_converter.geodetic2Enu(rtk_v[0], rtk_v[1], rtk_v[2], &x, &y, &z);
      p.x = x;
      p.y = y;
      p.z = z;
      rtk_pose->push_back(p);
      aff_rtk.setIdentity();
      aff_rtk.prerotate(Eigen::Quaternionf(p.curvature, p.normal_x, p.normal_y, p.normal_z));
      aff_rtk.pretranslate(Eigen::Vector3f(p.x, p.y, p.z));
      if (size_rtk == 1)
      {
        tf_first_inv = aff_rtk.inverse();
      }
      aff_rtk = tf_first_inv.matrix() * aff_rtk.matrix();
    }
    sensor_msgs::PointCloud2ConstPtr msg_cloud = m.instantiate<sensor_msgs::PointCloud2>();
    if (rtk_mapping_flag && msg_cloud != NULL)
    {
      if (lidar_msg_num % map_step == 0)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg_cloud, *cloud);
        pcl::PointCloud<pcl::PointXYZI> cloud_out;
        Eigen::Affine3f tf_to_rtk;
        tf_to_rtk = aff_rtk.matrix() * tf.matrix();
        pcl::transformPointCloud(*cloud, cloud_out, tf_to_rtk);
        std::string fname = "/home/qcl/bag/tmp/" + std::to_string(lidar_msg_num) + ".pcd";
        pcl::io::savePCDFileASCII(fname, cloud_out);
      }
      lidar_msg_num++;
    }
  }
  std::cout << "topics:\n"
            << imu_topic << "\t" << size_imu << " msg \n"
            << rtk_topic << "\t" << size_rtk << " msg \n"
            << std::endl;
  std::cout << "rtk pose size is " << rtk_pose->size() << std::endl;
  std::cout << "save rtk data -> " << rtk_pose_file << std::endl;
  pcl::io::savePCDFileASCII(rtk_pose_file, *rtk_pose);
  ros::shutdown();
  return 0;
}
