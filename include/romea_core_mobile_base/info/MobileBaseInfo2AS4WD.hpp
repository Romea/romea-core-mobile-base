#ifndef romea_MobileBaseDescription2AS4WD_hpp
#define romea_MobileBaseDescription2AS4WD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2AS4WD
{
  MobileBaseInfo2AS4WD();
  TwoAxlesGeometry<Wheel,Wheel> geometry;
  SteeringAngleControl axlesSteeringControl;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2AS4WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2AS4WD & baseInformation,
                             TwoAxleSteeringKinematic::Parameters & kinematicParameters );

}

#endif
