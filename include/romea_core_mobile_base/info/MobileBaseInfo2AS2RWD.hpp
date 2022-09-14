#ifndef romea_MobileBaseDescription2AS2RWD_hpp
#define romea_MobileBaseDescription2AS2RWD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2AS2RWD
{
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl axlesSteeringControl;
  WheelSpeedControl rearWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2AS4WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2AS2RWD & baseInformation,
                             TwoAxleSteeringKinematic::Parameters & kinematicParameters );

}

#endif
