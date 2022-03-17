#ifndef romea_OdometryFrame2AS4WD_hpp
#define romea_OdometryFrame2AS4WD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {


struct OdometryFrame2AS4WD
{

  using Ptr = std::shared_ptr<OdometryFrame2AS4WD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame2AS4WD> ;

  double frontAxleSteeringAngle;
  double frontLeftWheelSpeed;
  double frontRightWheelSpeed;

  double rearAxleSteeringAngle;
  double rearLeftWheelSpeed;
  double rearRightWheelSpeed;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2AS4WD &frame);

}//end romea
#endif
