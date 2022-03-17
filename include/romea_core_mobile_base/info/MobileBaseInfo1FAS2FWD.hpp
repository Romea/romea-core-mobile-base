#ifndef romea_MobileBaseDescription1FAS2FWD_hpp
#define romea_MobileBaseDescription1FAS2FWD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo1FAS2FWD
{
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  SteeringAngleControl frontAxleSteeringControl;
  WheelSpeedControl frontWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2FWD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo1FAS2FWD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters );

}

#endif
