#ifndef romea_SkidSteeringMeasure_hpp
#define romea_SkidSteeringMeasure_hpp

//romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "../KinematicMeasure.hpp"

namespace romea {

struct SkidSteeringMeasure : public SkidSteeringCommand
{
  SkidSteeringMeasure();
  Eigen::Matrix2d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const SkidSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(const SkidSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(const SkidSteeringMeasure & measure,
                                    const SkidSteeringKinematic::Parameters & parameters);


}//end romea
#endif
