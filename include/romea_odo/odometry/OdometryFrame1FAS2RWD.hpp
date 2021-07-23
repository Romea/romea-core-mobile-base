#ifndef romea_OdometryFrame1FAS2RWD_hpp
#define romea_OdometryFrame1FAS2RWD_hpp

//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"

namespace romea {

struct OdometryFrame1FAS2RWD
{
  using Ptr =std::shared_ptr<OdometryFrame1FAS2RWD> ;
  using ConstPtr =std::shared_ptr<const OdometryFrame1FAS2RWD> ;

  double frontAxleSteeringAngle;
  double rearLeftWheelSpeed;
  double rearRightWheelSpeed;

//  static const inline std::vector<std::string> fields={"front_axle_steering_angle",
//                                                       "rear_left_wheel_speed",
//                                                       "rear_left_wheel_speed}
};

//void toOdometryFrame(const OdometryFrame1FAS2RWD & odometryFrame1FAS2RWD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame1FAS2RWD & odometryFrame1FAS2RWD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FAS2RWD &frame);

}//end romea
#endif
