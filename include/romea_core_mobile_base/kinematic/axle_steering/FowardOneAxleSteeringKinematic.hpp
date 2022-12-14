#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_FOWARDONEAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_FOWARDONEAXLESTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea {



void forwardKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame1FAS2FWD & odometryCommandFrame);

void forwardKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame1FAS2RWD & odometryCommandFrame);



}  // namespace romea

#endif   // ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_FOWARDONEAXLESTEERINGKINEMATIC_HPP_
