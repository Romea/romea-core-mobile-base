//romea
#include "romea_odo/odometry/OdometryFrame2FWS2RWD.hpp"

namespace romea {

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
