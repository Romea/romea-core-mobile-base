#ifndef romea_OdometryFrame2FWS2FWD_hpp
#define romea_OdometryFrame2FWS2FWD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame2FWS2FWD
{

  using Ptr = std::shared_ptr<OdometryFrame2FWS2FWD> ;
  using ConstPtr = std::shared_ptr<OdometryFrame2FWS2FWD> ;

  double frontLeftWheelSpeed;
  double frontLeftWheelAngle;
  double frontRightWheelSpeed;
  double frontRightWheelAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS2FWD &frame);

}//end romea
#endif
