//romea
#include "romea_odo/odometry/OdometryFrame1FAS2FWD.hpp"

//stl
#include <cassert>

namespace romea {

////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame1FAS2FWD & odometryFrame1FAS2FWD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_left_wheel_speed",odometryFrame1FAS2FWD.frontLeftWheelSpeed);
//  odometryFrame.set("front_right_wheel_speed",odometryFrame1FAS2FWD.frontRightWheelSpeed);
//  odometryFrame.set("front_axle_steering_axle",odometryFrame1FAS2FWD.frontAxleSteeringAngle);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FAS2FWD & odometryFrame1FAS2FWD)
//{
//  assert(odometryFrame.get("front_left_wheel_speed",odometryFrame1FAS2FWD.frontLeftWheelSpeed));
//  assert(odometryFrame.get("front_right_wheel_speed",odometryFrame1FAS2FWD.frontRightWheelSpeed));
//  assert(odometryFrame.get("front_axle_steering_angle",odometryFrame1FAS2FWD.frontAxleSteeringAngle));
//}


//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2FWD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  return s;
}

}//end romea
