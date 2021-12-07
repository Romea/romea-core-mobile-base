#ifndef romea_OdometryFrame4WD_hpp
#define romea_OdometryFrame4WD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame4WD
{

  using Ptr =std::shared_ptr<OdometryFrame4WD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame4WD> ;

  double frontLeftWheelSpeed;
  double frontRightWheelSpeed;
  double rearLeftWheelSpeed;
  double rearRightWheelSpeed;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame4WD &frame);


}//end romea

#endif
