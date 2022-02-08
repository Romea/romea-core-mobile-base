#ifndef romea_OdometryFrame2FWS2RWD_hpp
#define romea_OdometryFrame2FWS2RWD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame2FWS2RWD
{

  using Ptr = std::shared_ptr<OdometryFrame2FWS2RWD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame2FWS2RWD> ;

  double rearLeftWheelSpeed;
  double frontLeftWheelAngle;
  double rearRightWheelSpeed;
  double frontRightWheelAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS2RWD &frame);

}//end romea
#endif
