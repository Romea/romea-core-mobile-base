#ifndef romea_OdometryFrame1FAS4WD_hpp
#define romea_OdometryFrame1FAS4WD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame1FAS4WD
{
  double frontAxleSteeringAngle;
  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS4WD &frame);

}//end romea
#endif
