//romea
#include "romea_odo/odometry/OdometryFrame4WS4WD.hpp"

//std
#include <cassert>
#include <iostream>

namespace romea {


//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame4WS4WD & odometryFrame4WS4WD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("front_left_wheel",odometryFrame4WS4WD.frontLeftWheelSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",odometryFrame4WS4WD.frontRightWheelSpeed);
  odometryFrame.setWheelSpeed("rear_left_wheel",odometryFrame4WS4WD.rearLeftWheelSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",odometryFrame4WS4WD.rearRightWheelSpeed);
  odometryFrame.setSteeringAngle("front_left_steering",odometryFrame4WS4WD.frontLeftWheelAngle);
  odometryFrame.setSteeringAngle("front_right_steering",odometryFrame4WS4WD.frontRightWheelAngle);
  odometryFrame.setSteeringAngle("rear_left_steering",odometryFrame4WS4WD.rearLeftWheelAngle);
  odometryFrame.setSteeringAngle("rear_right_steering",odometryFrame4WS4WD.rearRightWheelAngle);

}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame4WS4WD & odometryFrame4WS4WD)
{
  assert(odometryFrame.getWheelSpeeds().size()==4);
  assert(odometryFrame.getWheelSpeed("front_left_wheel",odometryFrame4WS4WD.frontLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("front_right_wheel",odometryFrame4WS4WD.frontRightWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_left_wheel",odometryFrame4WS4WD.rearLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_right_wheel",odometryFrame4WS4WD.rearRightWheelSpeed));
  assert(odometryFrame.getSteeringAngles().size()==4);
  assert(odometryFrame.getSteeringAngle("front_left_steering",odometryFrame4WS4WD.frontLeftWheelAngle));
  assert(odometryFrame.getSteeringAngle("front_right_steering",odometryFrame4WS4WD.frontRightWheelAngle));
  assert(odometryFrame.getSteeringAngle("rear_left_steering",odometryFrame4WS4WD.rearLeftWheelAngle));
  assert(odometryFrame.getSteeringAngle("rear_right_steering",odometryFrame4WS4WD.rearRightWheelAngle));
}



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
