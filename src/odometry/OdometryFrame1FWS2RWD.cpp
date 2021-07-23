//romea
#include "romea_odo/odometry/OdometryFrame1FWS2RWD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {


////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame1FWS2RWD & odometryFrame1FWS2RWD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_wheel_angle",odometryFrame1FWS2RWD.frontWheelAngle);
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame1FWS2RWD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame1FWS2RWD.rearRightWheelSpeed);
//}


////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FWS2RWD & odometryFrame1FWS2RWD)
//{
//  assert(odometryFrame.get("front_wheel_angle",odometryFrame1FWS2RWD.frontWheelAngle));
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame1FWS2RWD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame1FWS2RWD.rearRightWheelSpeed));
//}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FWS2RWD &frame)
{
  s<< "frontWheelAngle : "<< frame.frontWheelAngle<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  return s;
}


}//end romea
