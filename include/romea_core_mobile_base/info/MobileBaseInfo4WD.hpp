#ifndef romea_MobileBaseDescription4WD_hpp
#define romea_MobileBaseDescription4WD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo4WD
{
//  MobileBaseInfo4WD();
  TwoAxles<WheeledAxle,WheeledAxle> geometry;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};


//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo4WD & description);

void to_kinematic_parameters(const MobileBaseInfo4WD & baseInformation,
                             SkidSteeringKinematic::Parameters & kinematicParameters );

}

#endif
