#ifndef romea_OdometryFrame1FWS2RWD_hpp
#define romea_OdometryFrame1FWS2RWD_hpp


//stl
#include <ostream>

namespace romea {

struct OdometryFrame1FWS2RWD
{
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
  double frontWheelSteeringAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FWS2RWD &frame);
}//end romea
#endif
