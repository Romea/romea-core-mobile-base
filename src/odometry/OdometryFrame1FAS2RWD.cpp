//romea
#include "romea_odo/odometry/OdometryFrame1FAS2RWD.hpp"

//stl
#include <cassert>

namespace romea {


////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame1FAS2RWD & odometryFrame1FAS2RWD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame1FAS2RWD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame1FAS2RWD.rearRightWheelSpeed);
//  odometryFrame.set("front_axle_steering_angle",odometryFrame1FAS2RWD.frontAxleSteeringAngle);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FAS2RWD & odometryFrame1FAS2RWD)
//{
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame1FAS2RWD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame1FAS2RWD.rearRightWheelSpeed));
//  assert(odometryFrame.get("front_axle_steering_angle",odometryFrame1FAS2RWD.frontAxleSteeringAngle));
//}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2RWD &frame)
{
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  return s;
}

}//end romea
