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

//struct OdometryFrame1FAS2FWDa
//{
//  double frontAxleSteeringAngle;
//  double frontLeftWheelAngularSpeed;
//  double frontRightWheelAngularSpeed;
//};

//std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2FWDa &frame);

}//end romea
#endif
