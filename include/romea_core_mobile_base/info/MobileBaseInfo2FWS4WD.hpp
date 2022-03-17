#ifndef romea_MobileBaseDescription2FWS4WD_hpp
#define romea_MobileBaseDescription2FWS4WD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2FWS4WD
{
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl frontWheelsSteeringControl;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2FWS4WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2FWS4WD & baseInformation,
                             TwoWheelSteeringKinematic::Parameters & kinematicParameters );

}

#endif
