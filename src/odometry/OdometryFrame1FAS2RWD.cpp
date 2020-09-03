//romea
#include "romea_odo/odometry/OdometryFrame1FAS2RWD.hpp"

//stl
#include <cassert>

namespace romea {


//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame1FAS2RWD & odometryFrame1FAS2RWD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("rear_left_wheel",odometryFrame1FAS2RWD.rearLeftWheelSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",odometryFrame1FAS2RWD.rearRightWheelSpeed);
  odometryFrame.setSteeringAngle("front_steering",odometryFrame1FAS2RWD.frontAxleSteeringAngle);
}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FAS2RWD & odometryFrame1FAS2RWD)
{
  assert(odometryFrame.getWheelSpeed("rear_left_wheel",odometryFrame1FAS2RWD.rearLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_right_wheel",odometryFrame1FAS2RWD.rearRightWheelSpeed));
  assert(odometryFrame.getSteeringAngle("front_steering",odometryFrame1FAS2RWD.frontAxleSteeringAngle));
}


//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2RWD &frame)
{
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  return s;
}

}//end romea
