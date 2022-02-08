//romea
#include "romea_core_mobile_base/odometry/OdometryFrame1FWS2RWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FWS2RWD &frame)
{
  s<< "frontWheelAngle : "<< frame.frontWheelAngle<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  return s;
}


}//end romea
