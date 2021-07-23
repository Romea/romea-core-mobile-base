//romea
#include "romea_odo/odometry/OdometryFrame2FWS2FWD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {

////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame2FWS2FWD & odometryFrame2FWS2FWD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_left_wheel_speed",odometryFrame2FWS2FWD.frontLeftWheelSpeed);
//  odometryFrame.set("front_right_wheel_speed",odometryFrame2FWS2FWD.frontRightWheelSpeed);
//  odometryFrame.set("front_left_wheel_angle",odometryFrame2FWS2FWD.frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",odometryFrame2FWS2FWD.frontRightWheelAngle);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS2FWD & odometryFrame2FWS2FWD)
//{
//  assert(odometryFrame.get("front_left_wheel_speed",odometryFrame2FWS2FWD.frontLeftWheelSpeed));
//  assert(odometryFrame.get("front_right_wheel_speed",odometryFrame2FWS2FWD.frontRightWheelSpeed));
//  assert(odometryFrame.get("front_left_wheel_angle",odometryFrame2FWS2FWD.frontLeftWheelAngle));
//  assert(odometryFrame.get("front_right_wheel_angle",odometryFrame2FWS2FWD.frontRightWheelAngle));
//}


//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS2FWD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontLeftWheelAngle : "<< frame.frontLeftWheelAngle<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "frontRightWheelAngle : "<<frame.frontRightWheelAngle<<std::endl;
  return s;
}


}//end romea
