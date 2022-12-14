#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_WHEEL_STEERING_INVERSEFOURWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_WHEEL_STEERING_INVERSEFOURWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WS4WD.hpp"

namespace romea {


void inverseKinematic(const FourWheelSteeringKinematic::Parameters &parameters,
                      const OdometryFrame4WS4WD & odometryFrame,
                      TwoAxleSteeringMeasure & twoAxleSteeringMeasure);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_WHEEL_STEERING_INVERSEFOURWHEELSTEERINGKINEMATIC_HPP_
