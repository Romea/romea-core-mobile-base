// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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
#include "romea_core_mobile_base/simulation/SimulationControl2FWC2RWD.hpp"

namespace romea {
namespace core {

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os,
                         const SimulationCommand2FWC2RWD& command) {
  os << " Simulation2FWC2RWD command : " << std::endl;
  os << " rear left wheel spinning setpoint : ";
  os << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : ";
  os << command.rearRightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os,
                         const SimulationState2FWC2RWD& state) {
  os << " Simulation2FWC2RWD state : " << std::endl;
  os << " front left wheel swiveling angle : ";
  os << state.frontLeftWheelSwivelingAngle << std::endl;
  os << " front right wheel swiveling angle : ";
  os << state.frontRightWheelSwivelingAngle << std::endl;
  os << " front left wheel spinning motion : ";
  os << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel spinning motion : ";
  os << state.frontRightWheelSpinningMotion << std::endl;
  os << " rear left wheel spinning motion : ";
  os << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel spinning motion : ";
  os << state.rearRightWheelSpinningMotion << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
SimulationCommand2FWC2RWD toSimulationCommand2FWC2RWD(
    const HardwareCommand2FWC2RWD& hardwareCommand) {
  return {hardwareCommand.rearLeftWheelSpinningSetPoint,
          hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationState2FWC2RWD toSimulationState2FWC2RWD(
  const double & /* wheelbase */,
  const double & /* frontTrack */,
  const double & /* frontWheelXOffset */,
  const double & /* frontWheelRadius */,
  const double & /* rearWheelRadius */,
  const HardwareState2FWC2RWD & hardwareState)
{
  SimulationState2FWC2RWD simulationState{};
  simulationState.frontLeftWheelSwivelingAngle =
      hardwareState.frontLeftWheelSwivelingAngle;
  simulationState.frontRightWheelSwivelingAngle =
      hardwareState.frontRightWheelSwivelingAngle;
  // Front caster wheel speeds are not reconstructed here. On the real robot,
  // with measured caster angles, they can be estimated by projection:
  //   v_l = rear_left_wheel_velocity * rear_wheel_radius
  //   v_r = rear_right_wheel_velocity * rear_wheel_radius
  //   v_x = (v_r + v_l) / 2
  //   omega_z = (v_r - v_l) / rear_track
  //   x_left = wheelbase + front_wheel_x_offset
  //   y_left = front_track / 2
  //   x_right = wheelbase + front_wheel_x_offset
  //   y_right = -front_track / 2
  //   v_contact(x, y) = {v_x - omega_z * y, omega_z * x}
  // where x and y are the caster wheel contact position in the body frame, and
  // theta is the measured caster wheel angle.
  //   rolling_direction(theta) = {cos(theta), sin(theta)}
  //   wheel_speed = dot(v_contact, rolling_direction) / front_wheel_radius
  //   lateral_slip_speed = -sin(theta) * v_contact.x + cos(theta) * v_contact.y
  // The reconstructed wheel speed should only be trusted when
  // lateral_slip_speed is small, because caster wheels do not align instantly
  // during direction changes.
  simulationState.rearLeftWheelSpinningMotion =
      hardwareState.rearLeftWheelSpinningMotion;
  simulationState.rearRightWheelSpinningMotion =
      hardwareState.rearRightWheelSpinningMotion;

  return simulationState;
}

//-----------------------------------------------------------------------------
HardwareState2FWC2RWD toHardwareState2FWC2RWD(
    const SimulationState2FWC2RWD& simulationState) {
  return {simulationState.frontLeftWheelSwivelingAngle,
          simulationState.frontRightWheelSwivelingAngle,
          simulationState.rearLeftWheelSpinningMotion,
          simulationState.rearRightWheelSpinningMotion};
}

}  // namespace core
}  // namespace romea
