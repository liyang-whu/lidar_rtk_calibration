#include "gq_lidar_rtk_calib.h"
#include "lidar_rtk_calibConfig.h"
#include "geodetic_conv.hpp"
#include "calib/HandEyeCalibration.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

#define foreach BOOST_FOREACH
#define DEBUG 0
#define SAVE_RTK_LIDAR_POSE 1
int static const MIN_CALIB_PAIR_NO = 10;
std::string dir_test = "/home/qcl/work_code/calibration/lidar_rtk_calib_ws/";
using namespace cicv;

template <typename Input>
Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat)
{
  Eigen::AngleAxisd ax3d(eigenQuat);
  return ax3d.angle() * ax3d.axis();
}

LidRtkCaib::LidRtkCaib()
{
  version();
}

void LidRtkCaib::version()
{
  std::cout << std::endl;
  std::cerr << "\033[1;32m~~ ================================================ ~~\033[0m" << std::endl;
  std::cerr << "\033[1;32m~~ ===== CICV Lidar RTK Calibation Tool  ===== ~~\033[0m" << std::endl;
  std::cerr << "\033[1;32m~~ ================================================ ~~\033[0m" << std::endl;
  std::cout << std::setw(30) << std::left << "\033[1;32mROS version: " << ROS_VERSION_MAJOR << "." << ROS_VERSION_MINOR
            << "." << ROS_VERSION_PATCH << "\033[0m" << std::endl;
  std::cout << std::setw(30) << std::left << "\033[1;32mPCL version: " << PCL_VERSION_PRETTY << "\033[0m" << std::endl;
  std::cout << std::setw(30) << std::left << "\033[1;32mBoost version: " << BOOST_LIB_VERSION << "\033[0m" << std::endl;
  std::cout << std::setw(30) << std::left << "\033[1;32mEigen version: " << EIGEN_WORLD_VERSION << "."
            << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << "\033[0m" << std::endl;
  std::cout << std::setw(30) << std::left
            << "\033[1;32mlidar_rtk_calibration version: " << LIDAR_RTK_CALIB_VERSION_MAJOR << "."
            << LIDAR_RTK_CALIB_VERSION_MINOR << "." << LIDAR_RTK_CALIB_VERSION_PATCH << "\033[0m" << std::endl;
  std::cout << "\n" << std::endl;
}

void LidRtkCaib::setParas(const std::string f_bag, const std::string rtk_t, const std::string imu_t,
                          const std::string lidar_t)
{
  fname_bag_ = f_bag;
  imu_topic_ = imu_t;
  rtk_topic_ = rtk_t;
  lidar_topic_ = lidar_t;
}

void LidRtkCaib::setLidarPose(const std::string f_lidar_pose, const int frame_start)
{
  fname_lidar_pose_ = f_lidar_pose;
  lidar_start_frame_ = frame_start;
}

Eigen::Affine3d LidRtkCaib::calib()
{
  getRtkPose();
  readLidarPose();
  synTopic();
  // calibration
  // cicv::HandEyeCalibration calib;
  // Eigen::Matrix4d result;
  // ceres::Solver::Summary summary;
  // calib.setVerbose(true);
  // std::cout <<"ev_rtk_pose: "<<ev_rtk_pose_.size()<<" ev_lidar_pose: "<<ev_lidar_pose_.size()<<std::endl;
  // calib.estimateHandEye(ev_rtk_pose_, ev_lidar_pose_, result, summary);
  // std::cout << "Lidar Frame relative to RTK Frame\n" << result << std::endl;

  // Eigen::Transform<double, 3, Eigen::Affine> resultAffine(result);
  // Eigen::Vector3d xyz = resultAffine.translation();
  // std::cout << "Translation (x,y,z) : " << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;

  // Eigen::Quaternion<double> quaternionResult(resultAffine.rotation());
  // std::stringstream ss;
  // ss << quaternionResult.w() << ", " << quaternionResult.x() << ", " << quaternionResult.y() << ", "
  //    << quaternionResult.z() << std::endl;
  // std::cout << "Rotation (w,x,y,z): " << ss.str() << std::endl;

  // Eigen::Transform<double, 3, Eigen::Affine> resultAffineInv = resultAffine.inverse();
  // xyz = resultAffineInv.translation();
  // std::cout << "Inverted Translation (x,y,z) : " << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
  // quaternionResult = Eigen::Quaternion<double>(resultAffineInv.rotation());
  // std::cerr << "Inverted rotation (w,x,y,z): " << quaternionResult.w() << " " << quaternionResult.x() << " "
  //           << quaternionResult.y() << " " << quaternionResult.z() << std::endl;

  if (ev_lidar_pose_.size() > MIN_CALIB_PAIR_NO)
  {
    return estimate();
  }
  return Eigen::Affine3d::Identity();
}

Eigen::Affine3d LidRtkCaib::estimate()
{
  auto t1_it = ev_rtk_pose_.begin();
  auto t2_it = ev_lidar_pose_.begin();

  Eigen::Affine3d firstRTKInverse = Eigen::Affine3d::Identity(), firstLidarInverse = Eigen::Affine3d::Identity();
  eigenVector tvecsRtk, rvecsRtk, tvecsLidarl, rvecsLidarl;

  bool firstTransform = true;

  for (std::size_t i = 0; i < ev_rtk_pose_.size(); ++i, ++t1_it, ++t2_it)
  {
    auto& eigenRTK = *t1_it;
    auto& eigenLidar = *t2_it;
    if (firstTransform)
    {
      firstRTKInverse = eigenRTK.inverse();
      firstLidarInverse = eigenLidar.inverse();
      // ROS_INFO("Adding first transformation.");
      firstTransform = false;
    }
    else
    {
      Eigen::Affine3d rtkPoseinFirstTipBase = firstRTKInverse * eigenRTK;
      Eigen::Affine3d lidarInFirstLidarlBase = firstLidarInverse * eigenLidar;

      rvecsRtk.push_back(eigenRotToEigenVector3dAngleAxis(rtkPoseinFirstTipBase.rotation()));
      tvecsRtk.push_back(rtkPoseinFirstTipBase.translation());

      rvecsLidarl.push_back(eigenRotToEigenVector3dAngleAxis(lidarInFirstLidarlBase.rotation()));
      tvecsLidarl.push_back(lidarInFirstLidarlBase.translation());
    }
  }

  cicv::HandEyeCalibration calib;
  Eigen::Matrix4d result;
  calib.setVerbose(true);
  calib.estimateHandEyeScrew(rvecsRtk, tvecsRtk, rvecsLidarl, tvecsLidarl, result, false);
  std::cout << "Lidar Frame relative to RTK Frame\n" << result << std::endl;
  Eigen::Affine3d resultAffine(result);

  Eigen::Vector3d xyz = resultAffine.translation();
  std::cout << "Translation (x,y,z) : " << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
  Eigen::Quaterniond quaternionResult(resultAffine.rotation());
  std::stringstream ss;
  ss << quaternionResult.w() << ", " << quaternionResult.x() << ", " << quaternionResult.y() << ", "
     << quaternionResult.z() << std::endl;
  std::cout << "Rotation (w,x,y,z): " << ss.str() << std::endl;

  Eigen::Affine3d resultAffineInv = resultAffine.inverse();
  xyz = resultAffineInv.translation();
  std::cout << "Inverted Translation (x,y,z) : " << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
  quaternionResult = Eigen::Quaterniond(resultAffineInv.rotation());
  std::cerr << "Inverted rotation (w,x,y,z): " << quaternionResult.w() << " " << quaternionResult.x() << " "
            << quaternionResult.y() << " " << quaternionResult.z() << std::endl;
  return resultAffine;
}
/* * 数据对齐，格式化数据
 * */
void LidRtkCaib::synTopic()
{
  std::size_t pair_n = std::min(v_rtk_pose_.size(), v_lidar_pose_.size());
  std::cout << "sync topic pair_n: " << pair_n << std::endl;

  for (std::size_t i = 0; i < pair_n; ++i)
  {
    Eigen::Affine3d aff_rtk;
    aff_rtk.setIdentity();
    Eigen::Vector3d v;
    v[0] = v_rtk_pose_.at(i).pose.pose.position.x;
    v[1] = v_rtk_pose_.at(i).pose.pose.position.y;
    v[2] = v_rtk_pose_.at(i).pose.pose.position.z;
    Eigen::Quaterniond qua;
    qua.x() = v_rtk_pose_.at(i).pose.pose.orientation.x;
    qua.y() = v_rtk_pose_.at(i).pose.pose.orientation.y;
    qua.z() = v_rtk_pose_.at(i).pose.pose.orientation.z;
    qua.w() = v_rtk_pose_.at(i).pose.pose.orientation.w;
    aff_rtk.prerotate(qua);
    aff_rtk.pretranslate(v);

    Eigen::Affine3d aff_lidar;
    aff_lidar.setIdentity();
    v[0] = v_lidar_pose_.at(i).pose.pose.position.x;
    v[1] = v_lidar_pose_.at(i).pose.pose.position.y;
    v[2] = v_lidar_pose_.at(i).pose.pose.position.z;
    qua.x() = v_lidar_pose_.at(i).pose.pose.orientation.x;
    qua.y() = v_lidar_pose_.at(i).pose.pose.orientation.y;
    qua.z() = v_lidar_pose_.at(i).pose.pose.orientation.z;
    qua.w() = v_lidar_pose_.at(i).pose.pose.orientation.w;
    aff_lidar.prerotate(qua);
    aff_lidar.pretranslate(v);
    ev_rtk_pose_.push_back(aff_rtk);
    ev_lidar_pose_.push_back(aff_lidar);
  }
#if SAVE_RTK_LIDAR_POSE
  cv::FileStorage fs(dir_test + "rtk_lidar_matrix.yaml", cv::FileStorage::WRITE);
  if (fs.isOpened())
  {
    fs << "pair count " << (int)pair_n;
    for (std::size_t i = 0; i < pair_n; ++i)
    {
      cv::Mat_<double> t_rtk = cv::Mat_<double>::ones(4, 4);
      cv::eigen2cv(ev_rtk_pose_.at(i).matrix(), t_rtk);

      std::stringstream ss1;
      ss1 << "T_RTK_" << i;
      fs << ss1.str() << t_rtk;

      cv::Mat_<double> t_lidar = cv::Mat_<double>::ones(4, 4);
      cv::eigen2cv(ev_lidar_pose_.at(i).matrix(), t_lidar);

      std::stringstream ss2;
      ss2 << "T_LIDAR_" << i;
      fs << ss2.str() << t_lidar;
    }
    fs.release();
  }
#endif
}

void LidRtkCaib::getRtkPose()
{
  //需要解析的topics
  std::vector<std::string> topics;
  topics.push_back(imu_topic_);
  topics.push_back(lidar_topic_);
  topics.push_back(rtk_topic_);
  Eigen::Vector3d acc_0, gyr_0;
  geodetic_converter::GeodeticConverter g_geodetic_converter;
  rosbag::Bag bag;
  bag.open(fname_bag_, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double lidar_t_last_ = 0.0, imu_t_last_ = 0.0, rtk_t_last_ = 0.0;
  int size_lidar = 0, size_imu = 0, size_rtk = 0;
  geometry_msgs::PoseWithCovarianceStamped rtk_pose;
  geometry_msgs::PoseWithCovarianceStamped lidar_pose;

  int diff_time_size = 0;
  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
    if (imu_msg != NULL)
    {
      double t_now = imu_msg->header.stamp.toSec();
      // 检查IMU的时间
      // cout << t_now << " " << imu_t_last_ << "\t" << t_now - imu_t_last_ << std::endl;
      // ROS_ASSERT(t_now - imu_t_last_ >= 1e-3);
      imu_t_last_ = t_now;
      size_imu++;
      rtk_pose.pose.pose.orientation.x = imu_msg->orientation.x;
      rtk_pose.pose.pose.orientation.y = imu_msg->orientation.y;
      rtk_pose.pose.pose.orientation.z = imu_msg->orientation.z;
      rtk_pose.pose.pose.orientation.w = imu_msg->orientation.w;
      rtk_pose.pose.covariance[21] = imu_msg->orientation_covariance[0];
      rtk_pose.pose.covariance[28] = imu_msg->orientation_covariance[4];
      rtk_pose.pose.covariance[35] = imu_msg->orientation_covariance[8];
    }
    sensor_msgs::NavSatFixConstPtr rtk_msg = m.instantiate<sensor_msgs::NavSatFix>();
    if (rtk_msg != NULL)
    {
      double t_now = rtk_msg->header.stamp.toSec();
      // 检查RTK的时间
      // cout << t_now << " " << rtk_t_last_ << "\t" << t_now - rtk_t_last_ << std::endl;
      // ROS_ASSERT(t_now - rtk_t_last_ >= 0);
      rtk_t_last_ = t_now;
      // ROS_ASSERT(fabs(rtk_t_last_ - lidar_t_last_) < 0.02);
      Eigen::Vector3d rtk_v;
      rtk_v << rtk_msg->latitude, rtk_msg->longitude, rtk_msg->altitude;

      if (size_rtk == 0)
      {
        g_geodetic_converter.initialiseReference(rtk_v[0], rtk_v[1], rtk_v[2]);
      }

      size_rtk++;
      double x = 0, y = 0, z = 0;
      g_geodetic_converter.geodetic2Enu(rtk_v[0], rtk_v[1], rtk_v[2], &x, &y, &z);
      rtk_pose.header = rtk_msg->header;
      rtk_pose.header.frame_id = "odom";
      rtk_pose.pose.pose.position.x = x;
      rtk_pose.pose.pose.position.y = y;
      rtk_pose.pose.pose.position.z = z;
      rtk_pose.pose.covariance[0] = rtk_msg->position_covariance[0];
      rtk_pose.pose.covariance[7] = rtk_msg->position_covariance[4];
      rtk_pose.pose.covariance[14] = rtk_msg->position_covariance[8];
      // std::cout << x << ", " << y << ", " << z << "; ";
    }

    sensor_msgs::PointCloud2ConstPtr l_cloud = m.instantiate<sensor_msgs::PointCloud2>();
    if (l_cloud != NULL)
    {
      double t_now = l_cloud->header.stamp.toSec();
      // 检查雷达的时间
      // ROS_ASSERT(t_now - lidar_t_last_ >= 0);
      lidar_t_last_ = t_now;
      size_lidar++;
      lidar_pose.header = l_cloud->header;
      lidar_pose.header.frame_id = "odom";
      if (size_lidar > lidar_start_frame_)
      {
        if (std::fabs(lidar_t_last_ - rtk_t_last_) < 0.02 && std::fabs(lidar_t_last_ - imu_t_last_) < 0.02)
        {
          v_rtk_pose_.push_back(rtk_pose);
          v_lidar_pose_.push_back(lidar_pose);
        }
        else
        {
          ros::Time a_little_time(0.001);
          lidar_pose.header.stamp = a_little_time;  // lidar时间戳设置为0
          v_lidar_pose_.push_back(lidar_pose);
          diff_time_size++;
        }
      }
    }
  }
  bag.close();
  std::cout << "read topics:\n"
            << imu_topic_ << "\t" << size_imu << " msg \n"
            << lidar_topic_ << "\t" << size_lidar << " msg \n"
            << rtk_topic_ << "\t" << size_rtk << " msg \n"
            << std::endl;
  std::cout << "diff time size: " << diff_time_size << std::endl;
#if DEBUG
  bag.open(dir_test + "rtk_pose.bag", rosbag::bagmode::Write);
  for (std::size_t i = 0; i < v_rtk_pose_.size(); i++)
  {
    bag.write("rtk_pose", v_rtk_pose_.at(i).header.stamp, v_rtk_pose_.at(i));
  }
  bag.close();
#endif
  std::cout << "getRktPose end" << std::endl;
}
void LidRtkCaib::readLidarPose()
{
  pcl::PointCloud<pcl::PointNormal>::Ptr lidar_pose(new pcl::PointCloud<pcl::PointNormal>);
  if (pcl::io::loadPCDFile<pcl::PointNormal>(fname_lidar_pose_, *lidar_pose) == -1)
  {
    PCL_ERROR("Couldn't read lidar_pose file \n");
    return;
  }
  std::size_t min_size = std::min(v_lidar_pose_.size(), lidar_pose->size());
  std::size_t new_size = 0;
  std::cout << "lidar raw size " << v_lidar_pose_.size() << std::endl;
#if DEBUG
  std::cout << "lidar pose size " << lidar_pose->size() << std::endl;
  std::cout << "min size " << min_size << std::endl;
#endif
  for (std::size_t i = 0; i < min_size && i + lidar_start_frame_ < lidar_pose->size(); i++)
  {
    // std::cout << "time: " << std::setprecision(20) << v_lidar_pose_.at(i).header.stamp.toSec() << std::endl;
    if (v_lidar_pose_.at(i).header.stamp.toSec() > 1.0)
    {
      v_lidar_pose_.at(new_size).pose.pose.position.x = lidar_pose->at(i + lidar_start_frame_).x;
      v_lidar_pose_.at(new_size).pose.pose.position.y = lidar_pose->at(i + lidar_start_frame_).y;
      v_lidar_pose_.at(new_size).pose.pose.position.z = lidar_pose->at(i + lidar_start_frame_).z;
      v_lidar_pose_.at(new_size).pose.pose.orientation.x = lidar_pose->at(i + lidar_start_frame_).normal_x;
      v_lidar_pose_.at(new_size).pose.pose.orientation.y = lidar_pose->at(i + lidar_start_frame_).normal_y;
      v_lidar_pose_.at(new_size).pose.pose.orientation.z = lidar_pose->at(i + lidar_start_frame_).normal_z;
      v_lidar_pose_.at(new_size).pose.pose.orientation.w = lidar_pose->at(i + lidar_start_frame_).curvature;
      new_size++;
    }
  }
  v_lidar_pose_.resize(new_size);
  std::cout << "lidar new size " << new_size << std::endl;
  std::cout << "rtk size " << v_rtk_pose_.size() << std::endl;
  std::cout << "getLidarPose end" << std::endl;
}
