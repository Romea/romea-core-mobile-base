//romea
#include "romea_odo/odometry/OdometryFrame2AS4WD.hpp"

//stl
#include <cassert>

namespace romea {


////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame2AS4WD & odometryFrame2AS4WD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_left_wheel_speed",odometryFrame2AS4WD.frontLeftWheelSpeed);
//  odometryFrame.set("front_right_wheel_speed",odometryFrame2AS4WD.frontRightWheelSpeed);
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame2AS4WD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame2AS4WD.rearRightWheelSpeed);
//  odometryFrame.set("front_axle_steering_angle",odometryFrame2AS4WD.frontAxleSteeringAngle);
//  odometryFrame.set("rear_axle_steering_angle",odometryFrame2AS4WD.rearAxleSteeringAngle);
//}


////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2AS4WD & odometryFrame2AS4WD)
//{
//  assert(odometryFrame.get("front_left_wheel_speed",odometryFrame2AS4WD.frontLeftWheelSpeed));
//  assert(odometryFrame.get("front_right_wheel_speed",odometryFrame2AS4WD.frontRightWheelSpeed));
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame2AS4WD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame2AS4WD.rearRightWheelSpeed));
//  assert(odometryFrame.get("front_axle_steering_angle",odometryFrame2AS4WD.frontAxleSteeringAngle));
//  assert(odometryFrame.get("rear_axle_steering_angle",odometryFrame2AS4WD.rearAxleSteeringAngle));
//}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2AS4WD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  s<< "rearAxleSteeringAngle : "<<frame.rearAxleSteeringAngle<<std::endl;

  return s;
}

}//end romea
