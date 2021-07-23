#ifndef romea_OdometryFrame2FWS4WD_hpp
#define romea_OdometryFrame2FWS4WD_hpp

//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

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

//void toOdometryFrame(const OdometryFrame2FWS4WD & odometryFrame2FWS4WD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2FWS4WD & odometryFrame2FWS4WD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS4WD &frame);

}//end romea
#endif
