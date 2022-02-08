//romea
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WD &frame)
{
  s<< "frontLeftWheelSpeed : "<<frame.frontLeftWheelSpeed<<std::endl;
  s<< "frontRightWheelSpeed : "<< frame.frontRightWheelSpeed<<std::endl;
  s<< "rearLeftWheelSpeed : "<<frame.rearLeftWheelSpeed<<std::endl;
  s<< "rearRightWheelSpeed : "<<frame.rearRightWheelSpeed<<std::endl;
  return s;
}

}//end romea
