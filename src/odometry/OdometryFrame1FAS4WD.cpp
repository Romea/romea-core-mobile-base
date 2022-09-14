//romea
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS4WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS4WD &frame)
{
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  s<< "frontLeftWheelLinearSpeed : "<<frame.frontLeftWheelLinearSpeed<<std::endl;
  s<< "frontRightWheelLinearSpeed : "<<frame.frontRightWheelLinearSpeed<<std::endl;
  s<< "rearLeftWheelLinearSpeed : "<<frame.rearLeftWheelLinearSpeed<<std::endl;
  s<< "rearRightWheelLinearSpeed : "<<frame.rearRightWheelLinearSpeed<<std::endl;
  return s;
}

}//end romea
