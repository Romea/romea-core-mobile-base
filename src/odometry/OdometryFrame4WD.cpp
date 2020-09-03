//romea
#include "romea_odo/odometry/OdometryFrame4WD.hpp"

//stl
#include <cassert>

namespace romea {


//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame4WD & odometryFrame4WD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("front_left_wheel",odometryFrame4WD.frontLeftWheelSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",odometryFrame4WD.frontRightWheelSpeed);
  odometryFrame.setWheelSpeed("rear_left_wheel",odometryFrame4WD.rearLeftWheelSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",odometryFrame4WD.rearRightWheelSpeed);
}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame4WD & odometryFrame4WD)
{
  assert(odometryFrame.getWheelSpeed("front_left_wheel",odometryFrame4WD.frontLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("front_right_wheel",odometryFrame4WD.frontRightWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_left_wheel",odometryFrame4WD.rearLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_right_wheel",odometryFrame4WD.rearRightWheelSpeed));
}

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
