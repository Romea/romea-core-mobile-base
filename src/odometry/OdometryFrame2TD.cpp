//romea
#include "romea_core_mobile_base/odometry/OdometryFrame2TD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2TD &frame)
{
  s<< "leftTrackLinearSpeed : "<<frame.leftTrackLinearSpeed<<std::endl;
  s<< "rightTrackLinearSpeed : "<< frame.rightTrackLinearSpeed<<std::endl;
  return s;
}

////--------------------------------------------------------------------------
//std::ostream & operator<<(std::ostream & s, const OdometryFrame2TDa &frame)
//{
//  s<< "leftTrackAngularSpeed : "<<frame.leftTrackAngularSpeed<<std::endl;
//  s<< "rightTrackAngularSpeed : "<< frame.rightTrackAngularSpeed<<std::endl;
//  return s;
//}

}//end romea
