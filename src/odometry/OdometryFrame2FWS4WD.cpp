//romea
#include "romea_odo/odometry/OdometryFrame2FWS4WD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {

////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame2FWS4WD & odometryFrame2FWS4WD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_left_wheel_speed",odometryFrame2FWS4WD.frontLeftWheelSpeed);
//  odometryFrame.set("front_right_wheel_speed",odometryFrame2FWS4WD.frontRightWheelSpeed);
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame2FWS4WD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame2FWS4WD.rearRightWheelSpeed);
//  odometryFrame.set("front_left_wheel_angle",odometryFrame2FWS4WD.frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",odometryFrame2FWS4WD.frontRightWheelAngle);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS4WD & odometryFrame2FWS4WD)
//{
//  assert(odometryFrame.get("front_left_wheel_speed",odometryFrame2FWS4WD.frontLeftWheelSpeed));
//  assert(odometryFrame.get("front_right_wheel_speed",odometryFrame2FWS4WD.frontRightWheelSpeed));
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame2FWS4WD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame2FWS4WD.rearRightWheelSpeed));
//  assert(odometryFrame.get("front_left_wheel_angle",odometryFrame2FWS4WD.frontLeftWheelAngle));
//  assert(odometryFrame.get("front_right_wheel_angle",odometryFrame2FWS4WD.frontRightWheelAngle));
//}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS4WD &frame)
{
  s<< "frontLeftWheelAngle : "<< frame.frontLeftWheelAngle<<std::endl;
  s<< "frontRightWheelAngle : "<<frame.frontRightWheelAngle<<std::endl;
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<<frame.frontRightWheelSpeed<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  return s;
}


}//end romea
