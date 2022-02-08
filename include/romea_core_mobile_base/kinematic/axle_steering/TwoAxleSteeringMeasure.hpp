#ifndef romea_TwoAxleSteeringMeasure_hpp
#define romea_TwoAxleSteeringMeasure_hpp

//romea
#include "TwoAxleSteeringCommand.hpp"
#include "TwoAxleSteeringKinematic.hpp"
#include "../wheel_steering/FourWheelSteeringKinematic.hpp"
#include "../KinematicMeasure.hpp"

//Eigen
#include <Eigen/Core>

namespace romea {


struct TwoAxleSteeringMeasure : public TwoAxleSteeringCommand
{
  TwoAxleSteeringMeasure();
  Eigen::Matrix3d covariance;
};


KinematicMeasure toKinematicMeasure(const TwoAxleSteeringMeasure & measure,
                                    const double & frontWheelBase,
                                    const double & rearWheelBase);

KinematicMeasure toKinematicMeasure(const TwoAxleSteeringMeasure & measure,
                                    const TwoAxleSteeringKinematic::Parameters & parameters);

KinematicMeasure toKinematicMeasure(const TwoAxleSteeringMeasure & measure,
                                    const FourWheelSteeringKinematic::Parameters & parameters);

std::ostream& operator<<(std::ostream& os, const TwoAxleSteeringMeasure & command);


}//end romea
#endif
