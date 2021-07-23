//romea
#include "romea_odo/odometry/OdometryFrame4WD.hpp"

//stl
#include <cassert>

namespace romea {


////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame4WD & odometryFrame4WD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("front_left_wheel_speed",odometryFrame4WD.frontLeftWheelSpeed);
//  odometryFrame.set("front_right_wheel_speed",odometryFrame4WD.frontRightWheelSpeed);
//  odometryFrame.set("rear_left_wheel_speed",odometryFrame4WD.rearLeftWheelSpeed);
//  odometryFrame.set("rear_right_wheel_speed",odometryFrame4WD.rearRightWheelSpeed);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame4WD & odometryFrame4WD)
//{
//  assert(odometryFrame.get("front_left_wheel_speed",odometryFrame4WD.frontLeftWheelSpeed));
//  assert(odometryFrame.get("front_right_wheel_speed",odometryFrame4WD.frontRightWheelSpeed));
//  assert(odometryFrame.get("rear_left_wheel_speed",odometryFrame4WD.rearLeftWheelSpeed));
//  assert(odometryFrame.get("rear_right_wheel_speed",odometryFrame4WD.rearRightWheelSpeed));
//}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  return s;
}

}//end romea
