#ifndef romea_OdometryFrame2AS4WD_hpp
#define romea_OdometryFrame2AS4WD_hpp


//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

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

//void toOdometryFrame(const OdometryFrame2AS4WD & odometryFrame2AS4WD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2AS4WD & odometryFrame2AS4WD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame2AS4WD &frame);

}//end romea
#endif
