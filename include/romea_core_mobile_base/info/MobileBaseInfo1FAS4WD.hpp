#ifndef romea_MobileBaseDescription1FAS4WD_hpp
#define romea_MobileBaseDescription1FAS4WD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo1FAS4WD
{
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl frontAxleSteeringControl;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2RWD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo1FAS4WD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters );

}

#endif
