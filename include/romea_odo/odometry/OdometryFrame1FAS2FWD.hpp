#ifndef romea_OdometryFrame1FAS2FWD_hpp
#define romea_OdometryFrame1FAS2FWD_hpp

//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

namespace romea {

struct OdometryFrame1FAS2FWD
{

  using Ptr = std::shared_ptr<OdometryFrame1FAS2FWD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame1FAS2FWD> ;

  double frontAxleSteeringAngle;
  double frontLeftWheelSpeed;
  double frontRightWheelSpeed;

};

//void toOdometryFrame(const OdometryFrame1FAS2FWD & odometryFrame1FAS2FWD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FAS2FWD & odometryFrame1FAS2FWD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2FWD &frame);

}//end romea
#endif
