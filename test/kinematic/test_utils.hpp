// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef TEST_KINEMATIC_TEST_UTILS_HPP_
#define TEST_KINEMATIC_TEST_UTILS_HPP_

#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"

void compareKinematicMeasure(const romea::KinematicMeasure & measure1,
                             const romea::KinematicMeasure & measure2)
{
  ASSERT_NEAR(measure1.longitudinalSpeed, measure2.longitudinalSpeed, 0.001);
  ASSERT_NEAR(measure2.lateralSpeed, measure2.lateralSpeed, 0.001);
  ASSERT_NEAR(measure2.instantaneousCurvature, measure2.instantaneousCurvature, 0.001);
  ASSERT_NEAR(measure2.angularSpeed, measure2.angularSpeed, 0.001);
  ASSERT_NEAR(0, (measure1.covariance.array() - measure2.covariance.array()).sum(), 0.001);
}


#endif  // TEST_KINEMATIC_TEST_UTILS_HPP_