#ifndef romea_InverseOmniSteeringKinematic_hpp
#define romea_InverseOmniSteeringKinematic_hpp

//romea
#include "MecanumWheelSteeringKinematic.hpp"
#include "OmniSteeringMeasure.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea
{

void inverseKinematic(const MecanumWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame4WD & odometryFrame,
                      OmniSteeringMeasure & measure);


}//end romea

#endif
