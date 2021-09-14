//romea
#include "romea_odo/odometry/OdometryFrame4WS4WD.hpp"

namespace romea {

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
