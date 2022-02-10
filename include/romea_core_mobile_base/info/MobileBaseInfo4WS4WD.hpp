#ifndef romea_MobileBaseDescription4WS4WD_hpp
#define romea_MobileBaseDescription4WS4WD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo4WS4WD
{
  MobileBaseInfo4WS4WD();
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl wheelsSteeringControl;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

std::ostream& operator<<(std::ostream& os, const MobileBaseInfo4WS4WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo4WS4WD & baseInformation,
                             FourWheelSteeringKinematic::Parameters & kinematicParameters );

}

#endif
