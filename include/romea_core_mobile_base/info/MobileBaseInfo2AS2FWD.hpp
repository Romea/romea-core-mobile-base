#ifndef romea_MobileBaseDescription2AS2FWD_hpp
#define romea_MobileBaseDescription2AS2FWD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2AS2FWD
{
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl axlesSteeringControl;
  WheelSpeedControl frontWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2AS4WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2AS2FWD & baseInformation,
                             TwoAxleSteeringKinematic::Parameters & kinematicParameters );

}

#endif
