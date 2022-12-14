#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_ONEAXLESTEERINGMEASURE_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_ONEAXLESTEERINGMEASURE_HPP_

// Eigen
#include <Eigen/Core>

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"


namespace romea {

struct OneAxleSteeringMeasure : OneAxleSteeringCommand
{
  OneAxleSteeringMeasure();

  Eigen::Matrix2d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const OneAxleSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
                                    const double & frontWheelBase,
                                    const double & rearWheelBase);

KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
                                    const OneAxleSteeringKinematic::Parameters & parameters);

KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
                                    const TwoWheelSteeringKinematic::Parameters & parameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_AXLE_STEERING_ONEAXLESTEERINGMEASURE_HPP_
