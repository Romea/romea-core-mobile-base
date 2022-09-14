#ifndef romea_OdometryFrame2AS2RWD_hpp
#define romea_OdometryFrame2AS2RWD_hpp

//stl
#include <ostream>

namespace romea {


struct OdometryFrame2AS2RWD
{
  double frontAxleSteeringAngle;
  double rearAxleSteeringAngle;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2AS2RWD &frame);

}//end romea
#endif
