#ifndef romea_OdometryFrame1FWS2RWD_hpp
#define romea_OdometryFrame1FWS2RWD_hpp


//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

namespace romea {

struct OdometryFrame1FWS2RWD
{
  using Ptr = std::shared_ptr<OdometryFrame1FWS2RWD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame1FWS2RWD> ;

  double rearLeftWheelSpeed;
  double rearRightWheelSpeed;
  double frontWheelAngle;

};

//void toOdometryFrame(const OdometryFrame1FWS2RWD & odometryFrame1FWS2RWD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FWS2RWD & odometryFrame1FWS2RWD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FWS2RWD &frame);
}//end romea
#endif
