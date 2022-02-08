#ifndef romea_MobileBaseDescription1FAS2RWD_hpp
#define romea_MobileBaseDescription1FAS2RWD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo1FAS2RWD
{
  MobileBaseInfo1FAS2RWD();
  TwoAxlesGeometry<Wheel,Wheel> geometry;
  SteeringAngleControl frontAxleSteeringControl;
  WheelSpeedControl rearWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2RWD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo1FAS2RWD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters );

}

#endif
