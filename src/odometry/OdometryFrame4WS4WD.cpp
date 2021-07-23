//romea
#include "romea_odo/odometry/OdometryFrame4WS4WD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {


////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame4WS4WD & odometryFrame4WS4WD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_left_wheel_speed",odometryFrame4WS4WD.frontLeftWheelSpeed);
//  odometryFrame.set("front_right_wheel_speed",odometryFrame4WS4WD.frontRightWheelSpeed);
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame4WS4WD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame4WS4WD.rearRightWheelSpeed);
//  odometryFrame.set("front_left_wheel_angle",odometryFrame4WS4WD.frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",odometryFrame4WS4WD.frontRightWheelAngle);
//  odometryFrame.set("rear_left_wheel_angle",odometryFrame4WS4WD.rearLeftWheelAngle);
//  odometryFrame.set("rear_right_wheel_angle",odometryFrame4WS4WD.rearRightWheelAngle);

//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame4WS4WD & odometryFrame4WS4WD)
//{
//  assert(odometryFrame.get("front_left_wheel_speed",odometryFrame4WS4WD.frontLeftWheelSpeed));
//  assert(odometryFrame.get("front_right_wheel_speed",odometryFrame4WS4WD.frontRightWheelSpeed));
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame4WS4WD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame4WS4WD.rearRightWheelSpeed));
//  assert(odometryFrame.get("front_left_wheel_angle",odometryFrame4WS4WD.frontLeftWheelAngle));
//  assert(odometryFrame.get("front_right_wheel_angle",odometryFrame4WS4WD.frontRightWheelAngle));
//  assert(odometryFrame.get("rear_left_wheel_angle",odometryFrame4WS4WD.rearLeftWheelAngle));
//  assert(odometryFrame.get("rear_right_wheel_angle",odometryFrame4WS4WD.rearRightWheelAngle));
//}



//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WS4WD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontLeftWheelAngle : "<< frame.frontLeftWheelAngle<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "frontRightWheelAngle : "<<frame.frontRightWheelAngle<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearLeftWheelAngle : "<<frame.rearLeftWheelAngle<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  s<< "rearRightWheelAngle : "<<frame.rearRightWheelAngle<<std::endl;
  return s;
}


}//end romea
