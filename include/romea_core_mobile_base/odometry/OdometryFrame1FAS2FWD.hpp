#ifndef romea_OdometryFrame1FAS2FWD_hpp
#define romea_OdometryFrame1FAS2FWD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame1FAS2FWD
{
  double frontAxleSteeringAngle;
  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2FWD &frame);

}//end romea
#endif
