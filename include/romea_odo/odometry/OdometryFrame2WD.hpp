#ifndef romea_OdometryFrame2WD_hpp
#define romea_OdometryFrame2WD_hpp

//stl
#include <memory>
#include <ostream>

//romea
#include "OdometryFrame.hpp"


namespace romea {

struct OdometryFrame2WD
{

public:

  using Ptr = std::shared_ptr<OdometryFrame2WD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame2WD> ;

  double leftWheelSpeed;
  double rightWheelSpeed;
};

//void toOdometryFrame(const OdometryFrame2WD & odometryFrame2WD, OdometryFrame & odometryFrame);

//void fromOdometryFrame(const OdometryFrame & odometryFrame,OdometryFrame2WD & odometryFrame2WD);

std::ostream & operator<<(std::ostream &s, const OdometryFrame2WD &frame);


}//end romea
#endif
