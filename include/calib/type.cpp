#include "types.h"

namespace robosense
{
namespace slam
{
Pose3d::Pose3d()
{
  q.setIdentity();
  p.setZero();
}

Pose3d::Pose3d(Eigen::Vector3d p1, Eigen::Quaterniond q1) : p(p1), q(q1)
{
}
Pose3d::Pose3d(Eigen::Affine3d tf)
{
  p = tf.translation();
  q = tf.rotation();
}

Eigen::Matrix4d Pose3d::toMatrix() const
{
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  tf.prerotate(q);
  tf.pretranslate(p);
  return tf.matrix();
}
Pose3d Pose3d::inverse(void) const
{
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  tf.prerotate(q);
  tf.pretranslate(p);
  Pose3d pose3d;
  pose3d.p = tf.translation();
  pose3d.q = tf.rotation();
  return pose3d;
}

std::istream& operator>>(std::istream& input, Pose3d& pose)
{
  input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >> pose.q.z() >> pose.q.w();
  // Normalize the quaternion to account for precision loss due to
  // serialization.
  pose.q.normalize();
  return input;
}

std::ostream& operator<<(std::ostream& output, Pose3d& pose)
{
  output << pose.q.w() << ", " << pose.q.x() << ", " << pose.q.y() << ", " << pose.q.z() << ", " << pose.p[0] << ", "
         << pose.p[1] << ", " << pose.p[2];
  return output;
}
std::istream& operator>>(std::istream& input, Constraint3d& constraint)
{
  Pose3d& t_be = constraint.t_be;
  input >> constraint.id_begin >> constraint.id_end >> t_be;

  for (int i = 0; i < 6 && input.good(); ++i)
  {
    for (int j = i; j < 6 && input.good(); ++j)
    {
      input >> constraint.information(i, j);
      if (i != j)
      {
        constraint.information(j, i) = constraint.information(i, j);
      }
    }
  }
  return input;
}

}  // namespace slam
}  // namespace robosense

