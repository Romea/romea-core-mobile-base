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


#include "romea_core_mobile_base/kinematic/omni_steering/InverseMecanumWheelSteeringKinematic.hpp"

namespace romea
{

void inverseKinematic(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame4WD & odometryFrame,
  OmniSteeringMeasure & omniSteeringMeasure)
{
  const double halfTrack = parameters.wheelTrack / 2.;
  const double halfWheebase = parameters.wheelbase / 2;

  omniSteeringMeasure.longitudinalSpeed = MecanumWheelSteeringKinematic::
    computeLongitudinalSpeed(
    odometryFrame.frontLeftWheelLinearSpeed,
    odometryFrame.frontRightWheelLinearSpeed,
    odometryFrame.rearLeftWheelLinearSpeed,
    odometryFrame.rearRightWheelLinearSpeed);

  omniSteeringMeasure.lateralSpeed = MecanumWheelSteeringKinematic::
    computeLateralSpeed(
    odometryFrame.frontRightWheelLinearSpeed,
    odometryFrame.rearLeftWheelLinearSpeed,
    odometryFrame.frontLeftWheelLinearSpeed,
    odometryFrame.rearRightWheelLinearSpeed);

  omniSteeringMeasure.angularSpeed =
    MecanumWheelSteeringKinematic::
    computeAngularSpeed(
    odometryFrame.frontRightWheelLinearSpeed,
    odometryFrame.rearLeftWheelLinearSpeed,
    odometryFrame.frontLeftWheelLinearSpeed,
    odometryFrame.rearRightWheelLinearSpeed,
    halfWheebase,
    halfTrack);


  Eigen::MatrixXd J = Eigen::MatrixXd::Constant(3, 4, 1);
  J(1, 0) = J(1, 3) = J(2, 1) = J(2, 2) = -1;
  J.row(2) /= (halfTrack + halfWheebase);
  omniSteeringMeasure.covariance = J * J.transpose();
}

}  // namespace romea
