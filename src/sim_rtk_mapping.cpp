#include "sim_rtk_mapping.h"
#include "geodetic_conv.hpp"
geodetic_converter::GeodeticConverter g_geodetic_converter;
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr lidar_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*lidar_msg, *cloud);
  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
    cloud->at(i).x *= 0.5;
    cloud->at(i).y *= 0.5;
    cloud->at(i).z *= 0.5;
  }
  cloud->header.frame_id = "rslidar";
  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*cloud.get(), output);
  output.header = lidar_msg->header;
  pub_lidar.publish(output);
}
void imuCallback(const sensor_msgs::ImuConstPtr imu_msg)
{
  imu_q.setX(imu_msg->orientation.x);
  imu_q.setY(imu_msg->orientation.y);
  imu_q.setZ(imu_msg->orientation.z);
  imu_q.setW(imu_msg->orientation.w);
  imu_q.normalize();
}
void rtkCallback(const sensor_msgs::NavSatFixConstPtr gps_msg)
{
  Eigen::Vector3d gps_v;
  gps_v << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude;
  if (size_gps == 0)
  {
    g_geodetic_converter.initialiseReference(gps_v[0], gps_v[1], gps_v[2]);
  }
  size_gps++;
  double x = 0, y = 0, z = 0;
  g_geodetic_converter.geodetic2Enu(gps_v[0], gps_v[1], gps_v[2], &x, &y, &z);
  geometry_msgs::TransformStamped transformStamped;

  ros::Time stamp = gps_msg->header.stamp;
  // stamp = stamp + ros::Duration(6);
  transformStamped.header.stamp = stamp;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";

  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;

  transformStamped.transform.rotation.x = imu_q.x();
  transformStamped.transform.rotation.y = imu_q.y();
  transformStamped.transform.rotation.z = imu_q.z();
  transformStamped.transform.rotation.w = imu_q.w();
  broadcaster_odom_base->sendTransform(transformStamped);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp = stamp;
  pose.pose.orientation.x = imu_q.x();
  pose.pose.orientation.y = imu_q.y();
  pose.pose.orientation.z = imu_q.z();
  pose.pose.orientation.w = imu_q.w();

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  rtk_path.poses.push_back(pose);
  rtk_path.header = pose.header;
  pub_rtk_path.publish(rtk_path);
  broadcaster_laser_base->sendTransform(
      tf::StampedTransform(tf::Transform(tf::Quaternion(quat_v[1], quat_v[2], quat_v[3], quat_v[0]),
                                         tf::Vector3(tranl[0], tranl[1], tranl[2])),
                           stamp, "base_link", "rslidar"));
}
