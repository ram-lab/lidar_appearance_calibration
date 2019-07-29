//
// Created by hyye on 3/16/18.
//

#ifndef PLLO_MATH_UTILS_H_
#define PLLO_MATH_UTILS_H_

#include <cmath>
#include <limits>

namespace mathutils {

template<typename T>
inline T RadToDeg(T rad) {
  return rad * 180.0 / M_PI;
}

template<typename T>
inline T NormalizeRad(T rad) {
  rad = fmod(rad + M_PI, 2 * M_PI);
  if (rad < 0) {
    rad += 2 * M_PI;
  }
  return rad - M_PI;
}

template<typename T>
inline T DegToRad(T deg) {
  return deg / 180.0 * M_PI;
}

template<typename T>
inline T NormalizeDeg(T deg) {
  deg = fmod(deg + 180.0, 360.0);
  if (deg < 0) {
    deg += 360.0;
  }
  return deg - 180.0;
}

inline bool RadLt(double a, double b) {
  return NormalizeRad(a - b) < 0;
}

inline bool RadGt(double a, double b) {
  return NormalizeRad(a - b) > 0;
}

template<typename PointT>
inline PointT ScalePoint(const PointT &p, float scale) {
  PointT p_o = p;
  p_o.x *= scale;
  p_o.y *= scale;
  p_o.z *= scale;
  return p_o;
}

template<typename PointT>
/**
 * @return Euler distance between a and b
 */
inline float CalcSquaredDiff(const PointT &a, const PointT &b) 
{
  float diff_x = a.x - b.x;
  float diff_y = a.y - b.y;
  float diff_z = a.z - b.z;
  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

template<typename PointT>
inline float CalcSquaredDiff(const PointT& a, const PointT& b, const float& wb)
{
  float diff_x = a.x - b.x * wb;
  float diff_y = a.y - b.y * wb;
  float diff_z = a.z - b.z * wb;
  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

template<typename PointT>
inline float CalcPointDistance(const PointT &p) 
{
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename PointT>
inline float CalcSquaredPointDistance(const PointT& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta)
{
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

} // namespance mathutils

#endif //PLLO_MATH_UTILS_H_
