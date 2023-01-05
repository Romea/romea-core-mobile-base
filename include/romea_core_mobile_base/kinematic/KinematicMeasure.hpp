// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__KINEMATICMEASURE_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__KINEMATICMEASURE_HPP_

// eigen
#include <Eigen/Core>

namespace romea
{

struct KinematicMeasure
{
  KinematicMeasure();
  double longitudinalSpeed;
  double lateralSpeed;
  double angularSpeed;
  double instantaneousCurvature;
  Eigen::Matrix4d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


std::ostream & operator<<(std::ostream & os, const KinematicMeasure & measure);

}  // namespace romea

#endif // ROMEA_CORE_MOBILE_BASE__KINEMATIC__KINEMATICMEASURE_HPP_
