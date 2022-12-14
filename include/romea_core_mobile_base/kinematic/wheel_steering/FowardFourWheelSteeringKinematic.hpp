#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_WHEEL_STEERING_FOWARDFOURWHEELSTEERINGKINEMATIC
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_WHEEL_STEERING_FOWARDFOURWHEELSTEERINGKINEMATIC

// romea
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WS4WD.hpp"

namespace romea {

void forwardKinematic(const FourWheelSteeringKinematic::Parameters & kinematic,
                      const TwoAxleSteeringCommand & commandFrame,
                      OdometryFrame4WS4WD & commandOdometryFrame);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_WHEEL_STEERING_FOWARDFOURWHEELSTEERINGKINEMATIC
