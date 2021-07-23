//romea
#include "romea_odo/odometry/OdometryFrame2FWS2RWD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {

////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame2FWS2RWD & odometryFrame2FWS2RWD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame2FWS2RWD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame2FWS2RWD.rearRightWheelSpeed);
//  odometryFrame.set("front_left_wheel_angle",odometryFrame2FWS2RWD.frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",odometryFrame2FWS2RWD.frontRightWheelAngle);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS2RWD & odometryFrame2FWS2RWD)
//{
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame2FWS2RWD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame2FWS2RWD.rearRightWheelSpeed));
//  assert(odometryFrame.get("front_left_wheel_angle",odometryFrame2FWS2RWD.frontLeftWheelAngle));
//  assert(odometryFrame.get("front_right_wheel_angle",odometryFrame2FWS2RWD.frontRightWheelAngle));
//}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS2RWD &frame)
{
  s<< "frontLeftWheelAngle : "<< frame.frontLeftWheelAngle<<std::endl;
  s<< "frontRightWheelAngle : "<<frame.frontRightWheelAngle<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  return s;
}


}//end romea
