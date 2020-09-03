//romea
#include "romea_odo/odometry/OdometryFrame1FAS2FWD.hpp"

//stl
#include <cassert>

namespace romea {

//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame1FAS2FWD & odometryFrame1FAS2FWD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("front_left_wheel",odometryFrame1FAS2FWD.frontLeftWheelSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",odometryFrame1FAS2FWD.frontRightWheelSpeed);
  odometryFrame.setSteeringAngle("front_steering",odometryFrame1FAS2FWD.frontAxleSteeringAngle);

}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FAS2FWD & odometryFrame1FAS2FWD)
{
  assert(odometryFrame.getWheelSpeed("front_left_wheel",odometryFrame1FAS2FWD.frontLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("front_right_wheel",odometryFrame1FAS2FWD.frontRightWheelSpeed));
  assert(odometryFrame.getSteeringAngle("front_steering",odometryFrame1FAS2FWD.frontAxleSteeringAngle));
}


//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2FWD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  return s;
}

}//end romea
