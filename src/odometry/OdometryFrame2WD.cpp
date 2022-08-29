//romea
#include "romea_core_mobile_base/odometry/OdometryFrame2WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2WD &frame)
{
  s<< "leftWheelLinearSpeed : "<<frame.leftWheelLinearSpeed<<std::endl;
  s<< "rightWheelLinearSpeed : "<< frame.rightWheelLinearSpeed<<std::endl;
  return s;
}

////--------------------------------------------------------------------------
//std::ostream & operator<<(std::ostream & s, const OdometryFrame2WDa &frame)
//{
//  s<< "leftWheelAngularSpeed : "<<frame.leftWheelAngularSpeed<<std::endl;
//  s<< "rightWheelAngularSpeed : "<< frame.rightWheelAngularSpeed<<std::endl;
//  return s;
//}

}//end romea
