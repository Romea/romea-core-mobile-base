#ifndef romea_OdometryFrame1FAS2FWD_hpp
#define romea_OdometryFrame1FAS2FWD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame1FAS2FWD
{

  using Ptr = std::shared_ptr<OdometryFrame1FAS2FWD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame1FAS2FWD> ;

  double frontAxleSteeringAngle;
  double frontLeftWheelSpeed;
  double frontRightWheelSpeed;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2FWD &frame);

}//end romea
#endif
