#ifndef romea_OdometryFrame4WS4WD_hpp
#define romea_OdometryFrame4WS4WD_hpp

//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

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

//void toOdometryFrame(const OdometryFrame4WS4WD & odometryFrame4WS4WD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame4WS4WD & odometryFrame4WS4WD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame4WS4WD &frame);

}//end romea
#endif
