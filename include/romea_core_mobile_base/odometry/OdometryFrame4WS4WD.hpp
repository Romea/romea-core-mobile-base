#ifndef romea_OdometryFrame4WS4WD_hpp
#define romea_OdometryFrame4WS4WD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {



struct OdometryFrame4WS4WD
{

public :

  using Ptr = std::shared_ptr<OdometryFrame4WS4WD> ;
  using ConstPtr = std::shared_ptr<OdometryFrame4WS4WD> ;

  double frontLeftWheelSpeed;
  double frontLeftWheelAngle;

  double frontRightWheelSpeed;
  double frontRightWheelAngle;

  double rearLeftWheelSpeed;
  double rearLeftWheelAngle;

  double rearRightWheelSpeed;
  double rearRightWheelAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame4WS4WD &frame);

}//end romea
#endif
