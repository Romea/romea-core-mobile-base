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


// romea
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
KinematicMeasure::KinematicMeasure()
: longitudinalSpeed(0),
  lateralSpeed(0),
  angularSpeed(0),
  instantaneousCurvature(0),
  covariance(Eigen::Matrix4d::Zero())
{
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const KinematicMeasure & measure)
{
  os << " Kinematic measure  " << std::endl;
  os << " measured longitudinal speed  " << measure.longitudinalSpeed << std::endl;
  os << " measured lateral speed " << measure.lateralSpeed << std::endl;
  os << " measured angular speed " << measure.angularSpeed << std::endl;
  os << " measured instantaneous curvature " << measure.instantaneousCurvature << std::endl;
  os << " measured covariance matrix " << std::endl;
  os << measure.covariance;
  return os;
}

}  // namespace core
}  // namespace romea
