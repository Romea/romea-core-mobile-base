#ifndef romea_OdometryFrame2FWS2RWD_hpp
#define romea_OdometryFrame2FWS2RWD_hpp

//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

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

//void toOdometryFrame(const OdometryFrame2FWS2RWD & odometryFrame2FWS2RWD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS2RWD & odometryFrame2FWS2RWD);


std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS2RWD &frame);

}//end romea
#endif
