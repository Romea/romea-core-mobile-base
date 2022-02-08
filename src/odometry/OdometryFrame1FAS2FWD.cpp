//romea
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2FWD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  return s;
}

}//end romea
