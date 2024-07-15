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


#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEGEOMETRY_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEGEOMETRY_HPP_

#include <iostream>
#include <vector>
#include <array>

namespace romea
{
namespace core
{

struct Wheel
{
  double radius;
  double width;
  double hubCarrierOffset;
};

struct TrackWheel
{
  double radius;
  double x;
  double z;
};

struct ContinuousTrack
{
  double width;
  double thickness;
  TrackWheel sprocketWheel;
  std::vector<TrackWheel> idlerWheels;
  std::vector<TrackWheel> rollerWheels;
};

struct WheeledAxle
{
  double wheelsDistance;
  Wheel wheels;
};

struct ContinuousTrackedAxle
{
  double tracksDistance;
  ContinuousTrack tracks;
};


template<typename FrontAxle, typename RearAxle>
struct TwoAxles
{
  double axlesDistance;
  FrontAxle frontAxle;
  RearAxle rearAxle;
};

using TwoWheeledAxles = TwoAxles<WheeledAxle, WheeledAxle>;


// std::ostream& operator<<(std::ostream& os, const Wheel & wheel);

// template<typename Wheel>
// std::ostream & operator<<(std::ostream & os, const OneAxleGeometry<Wheel> & oneAxleGeometry);

// template<typename FrontWheel, typename RearWheel>
// std::ostream & operator<<(
//   std::ostream & os, const TwoAxlesGeometry<FrontWheel, RearWheel> & twoAxlesGeometry);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEGEOMETRY_HPP_
