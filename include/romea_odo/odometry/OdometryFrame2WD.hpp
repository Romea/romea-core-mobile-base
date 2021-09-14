#ifndef romea_OdometryFrame2WD_hpp
#define romea_OdometryFrame2WD_hpp

//stl
#include <memory>
#include <ostream>

namespace romea {

struct OdometryFrame2WD
{

public:

  using Ptr = std::shared_ptr<OdometryFrame2WD> ;
  using ConstPtr =std::shared_ptr<OdometryFrame2WD> ;

  double leftWheelSpeed;
  double rightWheelSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2WD &frame);


}//end romea
#endif
