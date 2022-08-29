#ifndef romea_OdometryFrame4WD_hpp
#define romea_OdometryFrame4WD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame4WD
{
  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame4WD &frame);

//struct OdometryFrame4WDa
//{
//  double frontLeftWheelAngularSpeed;
//  double frontRightWheelAngularSpeed;
//  double rearLeftWheelAngularSpeed;
//  double rearRightWheelAngularSpeed;

//};

//std::ostream & operator<<(std::ostream &s, const OdometryFrame4WDa &frame);


}//end romea

#endif
