#include "calib/HandEyeCalibration.h"

#include <boost/throw_exception.hpp>
#include <iostream>

#include <ceres/ceres.h>
#include "calib/EigenUtils.h"
#include "calib/types.h"
double NOISE_X = 0.05, NOISE_Y = 0.05, NOISE_Z = 0.02;
double NOISE_QX = 0.0043633, NOISE_QY = 0.0043633, NOISE_QZ = 0.0174;  // roll 0.5 ,pitch 0.5,yaw 2;

namespace robosense
{
class PoseError
{
public:
  PoseError(const robosense::slam::Pose3d& pose_a, const robosense::slam::Pose3d& pose_b,
            const Eigen::Matrix<double, 6, 6>& sqrt_info)
    : pose_a_(pose_a), pose_b_(pose_b), sqrt_info_(sqrt_info)
  {
  }

  template <typename T>
  bool operator()(const T* const t3x1, const T* const q4x1, T* residuals_ptr) const  //优化的变量与残差
  {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> tf_p(t3x1);  // 优化目标
    Eigen::Map<const Eigen::Quaternion<T>> tf_q(q4x1);
    Eigen::Transform<T, 3, Eigen::Affine> tf;
    tf.setIdentity();
    tf.prerotate(tf_q);
    tf.pretranslate(tf_p);
    Eigen::Transform<T, 3, Eigen::Affine> T_a_est;
    Eigen::Matrix<T, 4, 4> t_temp = tf.matrix() * pose_b_.toMatrix().template cast<T>() *
                                    tf.inverse().matrix();  // T_a_est = * pose_b_.toMatrix() * template cast<T>()
    T_a_est = t_temp;

    Eigen::Quaternion<T> q_a_est;
    q_a_est = T_a_est.rotation();
    Eigen::Matrix<T, 3, 1> p_a_est = T_a_est.translation();
    // pose_a_est - pose_a_
    Eigen::Quaternion<T> delta_q = pose_a_.q.template cast<T>() * q_a_est.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals_ptr);
    residual.template block<3, 1>(0, 0) = p_a_est - pose_a_.p.template cast<T>();
    residual.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
    // Scale the residuals by the measurement uncertainty.
    residual.applyOnTheLeft(sqrt_info_.template cast<T>());

    return true;
  }

private:
  const robosense::slam::Pose3d pose_a_, pose_b_;  //采样值
  const Eigen::Matrix<double, 6, 6> sqrt_info_;

public:
  static ceres::CostFunction* Create(const slam::Pose3d& pose3d_a, const slam::Pose3d& pose3d_b,
                                     const Eigen::Matrix<double, 6, 6>& sqrt_info)
  {
    return new ceres::AutoDiffCostFunction<PoseError, 6, 3, 4>(new PoseError(pose3d_a, pose3d_b, sqrt_info));
  }
  /// @see
  /// http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

HandEyeCalibration::HandEyeCalibration()
{
  mVerbose = true;
}

void HandEyeCalibration::setVerbose(bool on)
{
  mVerbose = on;
}

/// docs in header
void HandEyeCalibration::estimateHandEye(
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& T_a,
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& T_b, Eigen::Matrix4d& tf,
    ceres::Solver::Summary& summary)
{
  std::size_t motionCount = std::min(T_a.size(), T_b.size());
  Eigen::MatrixXd T(motionCount * 6, 8);
  T.setZero();
  Eigen::Matrix4d T_a_inv = T_a.at(0).matrix().inverse();
  Eigen::Matrix4d T_b_inv = T_b.at(0).matrix().inverse();

  for (size_t i = 0; i < motionCount; ++i)
  {
    Eigen::Affine3d aff_a;
    aff_a = T_a_inv * T_a.at(i).matrix();

    robosense::slam::Pose3d pose3d_a;
    pose3d_a.p = aff_a.translation();
    pose3d_a.q = aff_a.rotation();
    pose_a_.push_back(pose3d_a);
    // std::cout << "aff_a_" << i << " : ";
    // std::cout << aff_a.matrix() << std::endl;

    // std::cout << pose3d_a << std::endl;

    Eigen::Affine3d aff_b;
    aff_b = T_b_inv * T_b.at(i).matrix();
    robosense::slam::Pose3d pose3d_b;
    pose3d_b.p = aff_b.translation();
    pose3d_b.q = aff_b.rotation();
    pose_b_.push_back(pose3d_b);

    // std::cout << "aff_b_" << i << " : ";
    // std::cout << aff_b.matrix() << std::endl;
    // std::cout << pose3d_b << std::endl;
  }

  // auto dq = estimateHandEyeScrewInitial(T, planarMotion);
  Eigen::Quaterniond q(1, 0, 0, 0);
  Eigen::Vector3d t;
  t << 0, 0, 0;
  robosense::slam::Pose3d pose3d_tf;

  tf = pose3d_tf.toMatrix();
  if (mVerbose)
  {
    std::cout << "# INFO: Before refinement: tf = " << std::endl;
    std::cout << tf << std::endl;
  }

  estimateHandEyeRefine(pose3d_tf, pose_a_, pose_b_, summary);

  tf = pose3d_tf.toMatrix();
  if (mVerbose)
  {
    std::cout << "# INFO: After refinement: tf = " << std::endl;
    std::cout << tf << std::endl;
  }
}

// docs in header
void HandEyeCalibration::estimateHandEyeRefine(
    robosense::slam::Pose3d& pose3d_tf,
    const std::vector<robosense::slam::Pose3d, Eigen::aligned_allocator<robosense::slam::Pose3d>>& pose3d_a,
    const std::vector<robosense::slam::Pose3d, Eigen::aligned_allocator<robosense::slam::Pose3d>>& pose3d_b,
    ceres::Solver::Summary& summary)
{
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  covariance(0, 0) = NOISE_X * NOISE_X;
  covariance(1, 1) = NOISE_Y * NOISE_Y;
  covariance(2, 2) = NOISE_Z * NOISE_X;
  covariance(3, 3) = NOISE_QX * NOISE_QX;
  covariance(4, 4) = NOISE_QY * NOISE_QY;
  covariance(5, 5) = NOISE_QZ * NOISE_QZ;
  const Eigen::Matrix<double, 6, 6> sqrt_information = covariance.inverse();

  ceres::Problem problem;
  ceres::LocalParameterization* quaternionParameterization = new ceres::EigenQuaternionParameterization;

  for (size_t i = 0; i < pose3d_a.size(); i++)
  {
    // ceres deletes the objects allocated here for the user
    ceres::CostFunction* costFunction = PoseError::Create(pose3d_a[i], pose3d_b[i], sqrt_information);

    problem.AddResidualBlock(costFunction, NULL, pose3d_tf.p.data(),
                             pose3d_tf.q.coeffs().data());  // ceres deletes the object allocated here for the user

    problem.SetParameterization(pose3d_tf.q.coeffs().data(), quaternionParameterization);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.jacobi_scaling = true;
  options.max_num_iterations = 500;

  // ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (mVerbose)
  {
    std::cout << summary.BriefReport() << std::endl;
  }
}
}  // namespace robosense
