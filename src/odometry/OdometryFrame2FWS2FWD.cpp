//romea
#include "romea_odo/odometry/OdometryFrame2FWS2FWD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {

//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame2FWS2FWD & odometryFrame2FWS2FWD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("front_left_wheel",odometryFrame2FWS2FWD.frontLeftWheelSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",odometryFrame2FWS2FWD.frontRightWheelSpeed);
  odometryFrame.setSteeringAngle("front_left_steering",odometryFrame2FWS2FWD.frontLeftWheelAngle);
  odometryFrame.setSteeringAngle("front_right_steering",odometryFrame2FWS2FWD.frontRightWheelAngle);
}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS2FWD & odometryFrame2FWS2FWD)
{
  assert(odometryFrame.getWheelSpeed("front_left_wheel",odometryFrame2FWS2FWD.frontLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("front_right_wheel",odometryFrame2FWS2FWD.frontRightWheelSpeed));
  assert(odometryFrame.getSteeringAngle("front_left_steering",odometryFrame2FWS2FWD.frontLeftWheelAngle));
  assert(odometryFrame.getSteeringAngle("front_right_steering",odometryFrame2FWS2FWD.frontRightWheelAngle));
}


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
