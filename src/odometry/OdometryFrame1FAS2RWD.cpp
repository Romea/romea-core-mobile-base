//romea
#include "romea_core_odo/odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2RWD &frame)
{
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  s<< "frontAxleSteeringAngle : "<<frame.frontAxleSteeringAngle<<std::endl;
  return s;
}

}//end romea
