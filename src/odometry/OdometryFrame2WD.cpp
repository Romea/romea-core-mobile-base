//romea
#include "romea_odo/odometry/OdometryFrame2WD.hpp"

//stl
#include <cassert>

namespace romea {


//--------------------------------------------------------------------------
void toOdometryFrame(const OdometryFrame2WD & odometryFrame2WD, OdometryFrame & odometryFrame)
{
  odometryFrame.setWheelSpeed("left_wheel",odometryFrame2WD.leftWheelSpeed);
  odometryFrame.setWheelSpeed("right_wheel",odometryFrame2WD.rightWheelSpeed);
}

//--------------------------------------------------------------------------
void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2WD & odometryFrame2WD)
{
  assert(odometryFrame.getWheelSpeed("left_wheel",odometryFrame2WD.leftWheelSpeed));
  assert(odometryFrame.getWheelSpeed("right_wheel",odometryFrame2WD.rightWheelSpeed));
}


//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2WD &frame)
{
  s<< "leftWheelSpeed : "<<frame.leftWheelSpeed<<std::endl;
  s<< "rightWheelSpeed : "<< frame.rightWheelSpeed<<std::endl;
  return s;
}

}//end romea
