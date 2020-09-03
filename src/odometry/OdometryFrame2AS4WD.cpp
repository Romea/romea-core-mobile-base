//romea
#include "romea_odo/odometry/OdometryFrame2AS4WD.hpp"

//stl
#include <cassert>

namespace romea {


//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame2AS4WD & odometryFrame2AS4WD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("front_left_wheel",odometryFrame2AS4WD.frontLeftWheelSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",odometryFrame2AS4WD.frontRightWheelSpeed);
  odometryFrame.setWheelSpeed("rear_left_wheel",odometryFrame2AS4WD.rearLeftWheelSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",odometryFrame2AS4WD.rearRightWheelSpeed);
  odometryFrame.setSteeringAngle("front_steering",odometryFrame2AS4WD.frontAxleSteeringAngle);
  odometryFrame.setSteeringAngle("rear_steering",odometryFrame2AS4WD.rearAxleSteeringAngle);
}


//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2AS4WD & odometryFrame2AS4WD)
{
  assert(odometryFrame.getWheelSpeed("front_left_wheel",odometryFrame2AS4WD.frontLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("front_right_wheel",odometryFrame2AS4WD.frontRightWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_left_wheel",odometryFrame2AS4WD.rearLeftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("rear_right_wheel",odometryFrame2AS4WD.rearRightWheelSpeed));
  assert(odometryFrame.getSteeringAngle("front_steering",odometryFrame2AS4WD.frontAxleSteeringAngle));
  assert(odometryFrame.getSteeringAngle("rear_steering",odometryFrame2AS4WD.rearAxleSteeringAngle));

}

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
