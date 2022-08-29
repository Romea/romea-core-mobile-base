#ifndef romea_OdometryFrame2AS4WD_hpp
#define romea_OdometryFrame2AS4WD_hpp

//stl
#include <ostream>

namespace romea {


struct OdometryFrame2AS4WD
{
  double frontAxleSteeringAngle;
  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;

  double rearAxleSteeringAngle;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2AS4WD &frame);

}//end romea
#endif
