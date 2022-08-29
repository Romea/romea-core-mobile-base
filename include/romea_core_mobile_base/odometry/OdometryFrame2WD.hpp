#ifndef romea_OdometryFrame2WD_hpp
#define romea_OdometryFrame2WD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame2WD
{
  double leftWheelLinearSpeed;
  double rightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2WD &frame);

//struct OdometryFrame2WDa
//{
//  double leftWheelAngularSpeed;
//  double rightWheelAngularSpeed;
//};

//std::ostream & operator<<(std::ostream &s, const OdometryFrame2WDa &frame);


}//end romea
#endif
