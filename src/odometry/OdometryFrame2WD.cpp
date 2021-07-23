//romea
#include "romea_odo/odometry/OdometryFrame2WD.hpp"

//stl
#include <cassert>

namespace romea {


////--------------------------------------------------------------------------
//void toOdometryFrame(const OdometryFrame2WD & odometryFrame2WD, OdometryFrame & odometryFrame)
//{
//  odometryFrame.set("left_wheel_speed",odometryFrame2WD.leftWheelSpeed);
//  odometryFrame.set("right_wheel_speed",odometryFrame2WD.rightWheelSpeed);
//}

////--------------------------------------------------------------------------
//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2WD & odometryFrame2WD)
//{
//  assert(odometryFrame.get("left_wheel_speed",odometryFrame2WD.leftWheelSpeed));
//  assert(odometryFrame.get("right_wheel_speed",odometryFrame2WD.rightWheelSpeed));
//}


//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2WD &frame)
{
  s<< "leftWheelSpeed : "<<frame.leftWheelSpeed<<std::endl;
  s<< "rightWheelSpeed : "<< frame.rightWheelSpeed<<std::endl;
  return s;
}

}//end romea
