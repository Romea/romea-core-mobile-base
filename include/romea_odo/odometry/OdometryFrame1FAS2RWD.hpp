#ifndef romea_OdometryFrame1FAS2RWD_hpp
#define romea_OdometryFrame1FAS2RWD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame1FAS2RWD
{
  using Ptr =std::shared_ptr<OdometryFrame1FAS2RWD> ;
  using ConstPtr =std::shared_ptr<const OdometryFrame1FAS2RWD> ;

  double frontAxleSteeringAngle;
  double rearLeftWheelSpeed;
  double rearRightWheelSpeed;
};


std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2RWD &frame);

}//end romea
#endif
