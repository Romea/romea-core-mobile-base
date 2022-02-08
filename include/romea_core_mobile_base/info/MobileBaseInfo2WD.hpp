#ifndef romea_MobileBaseDescription2WD_hpp
#define romea_MobileBaseDescription2WD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2WD
{
  MobileBaseInfo2WD();
  OneAxleGeometry<Wheel> geometry;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};


std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2WD & baseInformation,
                             SkidSteeringKinematic::Parameters & kinematicParameters );

}

#endif