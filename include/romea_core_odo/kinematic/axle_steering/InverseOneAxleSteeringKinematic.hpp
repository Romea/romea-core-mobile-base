#ifndef romea_InverseOneAxleSteeringKinematic_hpp
#define romea_InverseOneAxleSteeringKinematic_hpp

//romea
#include "OneAxleSteeringMeasure.hpp"
#include "OneAxleSteeringKinematic.hpp"
#include "OneAxleSteeringMeasure.hpp"
#include "../../odometry/OdometryFrame1FAS2FWD.hpp"
#include "../../odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea {

void inverseKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame1FAS2FWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure);

void inverseKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame1FAS2RWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure);

}

#endif
