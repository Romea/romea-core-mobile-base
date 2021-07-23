#ifndef romea_OdometryFrame4WD_hpp
#define romea_OdometryFrame4WD_hpp

//romea
#include "OdometryFrame.hpp"

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

//void toOdometryFrame(const OdometryFrame4WD & odometryFrame4WD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame4WD & odometryFrame4WD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame4WD &frame);


}//end romea

#endif
