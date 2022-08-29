//romea
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WD &frame)
{
  s<< "frontLeftWheelLinearSpeed : "<<frame.frontLeftWheelLinearSpeed<<std::endl;
  s<< "frontRightWheelLinearSpeed : "<< frame.frontRightWheelLinearSpeed<<std::endl;
  s<< "rearLeftWheelLinearSpeed : "<<frame.rearLeftWheelLinearSpeed<<std::endl;
  s<< "rearRightWheelLinearSpeed : "<<frame.rearRightWheelLinearSpeed<<std::endl;
  return s;
}

////--------------------------------------------------------------------------
//std::ostream & operator<<(std::ostream & s, const OdometryFrame4WDa &frame)
//{
//  s<< "frontLeftWheelAngularSpeed : "<<frame.frontLeftWheelAngularSpeed<<std::endl;
//  s<< "frontRightWheelAngularSpeed : "<< frame.frontRightWheelAngularSpeed<<std::endl;
//  s<< "rearLeftWheelAngularSpeed : "<<frame.rearLeftWheelAngularSpeed<<std::endl;
//  s<< "rearRightWheelAngularSpeed : "<<frame.rearRightWheelAngularSpeed<<std::endl;
//  return s;
//}

}//end romea
