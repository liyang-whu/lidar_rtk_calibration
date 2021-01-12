
#include "sim_rtk_mapping.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_imu_gps_calib");
  ros::NodeHandle nh("~");
  broadcaster_laser_base = new tf::TransformBroadcaster();
  broadcaster_odom_base = new tf::TransformBroadcaster();
  imu_q.setW(1);
  imu_q.setX(0);
  imu_q.setY(0);
  imu_q.setZ(0);
  std::string imu_topic;
  nh.getParam("imu_topic", imu_topic);

  std::string rtk_topic;
  nh.getParam("rtk_topic", rtk_topic);

  std::string lidar_topic;
  nh.getParam("lidar_topic", lidar_topic);

  nh.getParam("trans", tranl);
  nh.getParam("quat", quat_v);

  pub_rtk_path = nh.advertise<nav_msgs::Path>("rtk_path", 100);
  pub_lidar = nh.advertise<sensor_msgs::PointCloud2>("rslidar_points2", 10);
  ros::Subscriber rtk_sub = nh.subscribe(rtk_topic, 100, rtkCallback);
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imuCallback);
  ros::Subscriber lidar_sub = nh.subscribe(lidar_topic, 100, lidarCallback);
  ros::Rate rate(10);
  ros::spin();
  return 0;
}
