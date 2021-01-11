/*
 * =====================================================================================
 *
 *       Filename:  sim_rtk_mapping.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  01/17/2019 04:23:32 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
tf2::Quaternion imu_q;
ros::Publisher pub_rtk_path;
ros::Publisher pub_lidar;
nav_msgs::Path rtk_path;
int size_lidar = 0, size_imu = 0, size_gps = 0;
tf::TransformBroadcaster* broadcaster_laser_base;
tf::TransformBroadcaster* broadcaster_odom_base;
std::vector<double> tranl(3, 0);
std::vector<double> quat_v(4, 0);  // w,x,y,z

using namespace std;

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr lidar_msg);
void imuCallback(const sensor_msgs::ImuConstPtr imu_msg);
void rtkCallback(const sensor_msgs::NavSatFixConstPtr gps_msg);
