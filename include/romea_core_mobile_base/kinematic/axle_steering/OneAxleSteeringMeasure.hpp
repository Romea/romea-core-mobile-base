#ifndef romea_OneAxleSteeringMeasure_hpp
#define romea_OneAxleSteeringMeasure_hpp

//romea
#include "OneAxleSteeringCommand.hpp"
#include "OneAxleSteeringKinematic.hpp"
#include "../wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "../KinematicMeasure.hpp"

//Eigen
#include "Eigen/Core"

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

}//end romea
#endif
