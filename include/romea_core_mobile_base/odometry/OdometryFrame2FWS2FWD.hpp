#ifndef romea_OdometryFrame2FWS2FWD_hpp
#define romea_OdometryFrame2FWS2FWD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame2FWS2FWD
{
  double frontLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;
  double frontRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS2FWD &frame);

}//end romea
#endif
