//romea
#include "romea_odo/odometry/OdometryFrame2FWS2RWD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {

//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame2FWS2RWD & odometryFrame2FWS2RWD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("rear_left_wheel",odometryFrame2FWS2RWD.rearLeftWheelSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",odometryFrame2FWS2RWD.rearRightWheelSpeed);
  odometryFrame.setSteeringAngle("front_left_steering",odometryFrame2FWS2RWD.frontLeftWheelAngle);
  odometryFrame.setSteeringAngle("front_right_steering",odometryFrame2FWS2RWD.frontRightWheelAngle);
}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS2RWD & odometryFrame2FWS2RWD)
{
  assert(odometryFrame.getWheelSpeed("rear_left_wheel",odometryFrame2FWS2RWD.rearLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_right_wheel",odometryFrame2FWS2RWD.rearRightWheelSpeed));
  assert(odometryFrame.getSteeringAngle("front_left_steering",odometryFrame2FWS2RWD.frontLeftWheelAngle));
  assert(odometryFrame.getSteeringAngle("front_right_steering",odometryFrame2FWS2RWD.frontRightWheelAngle));
}

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
