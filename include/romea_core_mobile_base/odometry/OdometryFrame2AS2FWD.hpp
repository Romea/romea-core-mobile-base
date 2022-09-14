#ifndef romea_OdometryFrame2AS2FWD_hpp
#define romea_OdometryFrame2AS2FWD_hpp

//stl
#include <ostream>

namespace romea {


struct OdometryFrame2AS2FWD
{
  double frontAxleSteeringAngle;
  double rearAxleSteeringAngle;

  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2AS2FWD &frame);

}//end romea
#endif
