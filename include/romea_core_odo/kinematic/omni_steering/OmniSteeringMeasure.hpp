#ifndef romea_OmniSteeringMeasure_hpp
#define romea_OmniSteeringMeasure_hpp

//romea
#include "romea_core_odo/kinematic/KinematicMeasure.hpp"
#include "romea_core_odo/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_odo/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"

namespace romea {

struct OmniSteeringMeasure : public OmniSteeringCommand
{
  OmniSteeringMeasure();
  Eigen::Matrix3d covariance;
};

std::ostream& operator<<(std::ostream& os, const OmniSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(const OmniSteeringMeasure & measure);
KinematicMeasure toKinematicMeasure(const OmniSteeringMeasure & measure,
                                    const MecanumWheelSteeringKinematic::Parameters & parameters);


}//end romea
#endif
