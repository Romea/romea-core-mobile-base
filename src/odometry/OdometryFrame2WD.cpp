//romea
#include "romea_core_odo/odometry/OdometryFrame2WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2WD &frame)
{
  s<< "leftWheelSpeed : "<<frame.leftWheelSpeed<<std::endl;
  s<< "rightWheelSpeed : "<< frame.rightWheelSpeed<<std::endl;
  return s;
}

}//end romea
