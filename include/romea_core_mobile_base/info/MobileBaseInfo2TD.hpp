#ifndef romea_MobileBaseDescription2TD_hpp
#define romea_MobileBaseDescription2TD_hpp

#include "MobileBaseControl.hpp"
#include "MobileBaseGeometry.hpp"
#include "MobileBaseInertia.hpp"

#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2TD
{
  ContinuousTrackedAxle geometry;
  WheelSpeedControl tracksSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};


//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2TD & base_information);

void to_kinematic_parameters(const MobileBaseInfo2TD & base_information,
                             SkidSteeringKinematic::Parameters & kinematic_parameters );

}

#endif
