#ifndef romea_OdometryFrame1FAS2RWD_hpp
#define romea_OdometryFrame1FAS2RWD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame1FAS2RWD
{
  double frontAxleSteeringAngle;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2RWD &frame);


}//end romea
#endif
