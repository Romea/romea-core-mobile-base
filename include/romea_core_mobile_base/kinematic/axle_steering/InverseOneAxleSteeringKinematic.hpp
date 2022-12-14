#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_INVERSEONEAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_INVERSEONEAXLESTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea {

void inverseKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame1FAS2FWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure);

void inverseKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame1FAS2RWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_INVERSEONEAXLESTEERINGKINEMATIC_HPP_
