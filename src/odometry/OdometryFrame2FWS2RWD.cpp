//romea
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2RWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS2RWD &frame)
{
  s<< "frontLeftWheelSteeringAngle : "<< frame.frontLeftWheelSteeringAngle<<std::endl;
  s<< "frontRightWheelSteeringAngle : "<<frame.frontRightWheelSteeringAngle<<std::endl;
  s<< "rearLeftWheelLinearSpeed : "<<frame.rearLeftWheelLinearSpeed<<std::endl;
  s<< "rearRightWheelLinearSpeed : "<<frame.rearRightWheelLinearSpeed<<std::endl;
  return s;
}


}//end romea
