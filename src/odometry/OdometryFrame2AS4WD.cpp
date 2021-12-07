//romea
#include "romea_core_odo/odometry/OdometryFrame2AS4WD.hpp"

namespace romea {

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
