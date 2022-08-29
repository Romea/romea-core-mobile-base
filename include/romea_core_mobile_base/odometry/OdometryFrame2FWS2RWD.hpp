#ifndef romea_OdometryFrame2FWS2RWD_hpp
#define romea_OdometryFrame2FWS2RWD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame2FWS2RWD
{
  double rearLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;
  double rearRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS2RWD &frame);

}//end romea
#endif
