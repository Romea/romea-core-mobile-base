#ifndef romea_OdometryFrame2TD_hpp
#define romea_OdometryFrame2TD_hpp

//stl
#include <ostream>

namespace romea {

struct OdometryFrame2TD
{
  double leftTrackLinearSpeed;
  double rightTrackLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2TD &frame);

//struct OdometryFrame2TDa
//{
//  double leftTrackAngularSpeed;
//  double rightTrackAngularSpeed;
//};

//std::ostream & operator<<(std::ostream &s, const OdometryFrame2TDa &frame);


}//end romea
#endif
