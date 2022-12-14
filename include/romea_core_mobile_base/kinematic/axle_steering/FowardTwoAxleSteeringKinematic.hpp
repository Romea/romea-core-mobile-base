#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_FOWARDTWOAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_FOWARDTWOAXLESTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2AS4WD.hpp"

namespace romea {

void forwardKinematic(const TwoAxleSteeringKinematic::Parameters & parameters,
                      const TwoAxleSteeringCommand &commandFrame,
                      OdometryFrame2AS4WD & odometryCommandFrame);



}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_FOWARDTWOAXLESTEERINGKINEMATIC_HPP_
