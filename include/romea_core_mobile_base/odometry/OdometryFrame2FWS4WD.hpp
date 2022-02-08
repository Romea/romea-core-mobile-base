#ifndef romea_OdometryFrame2FWS4WD_hpp
#define romea_OdometryFrame2FWS4WD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame2FWS4WD
{

  using Ptr = std::shared_ptr<OdometryFrame2FWS4WD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame2FWS4WD> ;


  double frontLeftWheelSpeed;
  double rearLeftWheelSpeed;
  double frontLeftWheelAngle;
  double frontRightWheelSpeed;
  double rearRightWheelSpeed;
  double frontRightWheelAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS4WD &frame);

}//end romea
#endif
