// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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