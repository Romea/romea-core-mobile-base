//romea
#include "romea_odo/odometry/OdometryFrame2FWS2FWD.hpp"

namespace romea {

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
