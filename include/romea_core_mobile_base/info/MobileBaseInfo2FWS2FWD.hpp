#ifndef romea_MobileBaseDescription2FWS2FWD_hpp
#define romea_MobileBaseDescription2FWS2FWD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2FWS2FWD
{
//  MobileBaseInfo2FWS2FWD();
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl frontWheelsSteeringControl;
  WheelSpeedControl frontWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2FWS2FWD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2FWS2FWD & baseInformation,
                             TwoWheelSteeringKinematic::Parameters & kinematicParameters );

}

#endif
