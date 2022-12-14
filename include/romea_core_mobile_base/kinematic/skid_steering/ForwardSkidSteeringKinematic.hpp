#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_FORWARDSKIDSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_FORWARDSKIDSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2TD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2WD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea
{

void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2TD & odometryCommandFrame);


void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2WD & odometryCommandFrame);


void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame4WD & odometryCommandFrame);


void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      const OdometryFrame2WD & startOdometryFrame,
                      OdometryFrame2WD & odometryCommandFrame);


void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      const OdometryFrame2WD & startOdometryFrame,
                      OdometryFrame4WD & odometryCommandFrame);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_FORWARDSKIDSTEERINGKINEMATIC_HPP_
