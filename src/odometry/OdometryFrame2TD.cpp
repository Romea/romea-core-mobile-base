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


// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame2TD.hpp"

namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2TD & frame)
{
  s << "leftTrackLinearSpeed : " << frame.leftTrackLinearSpeed << std::endl;
  s << "rightTrackLinearSpeed : " << frame.rightTrackLinearSpeed << std::endl;
  return s;
}

}  // namespace core
}  // namespace romea
