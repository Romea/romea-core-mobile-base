#ifndef romea_OdometryFrame2FWS4WD_hpp
#define romea_OdometryFrame2FWS4WD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame2FWS4WD
{
  double frontLeftWheelLinearSpeed;
  double rearLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;
  double frontRightWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS4WD &frame);

}//end romea
#endif
