#ifndef romea_OneAxleSteeringConstraints_hpp
#define romea_OneAxleSteeringConstraints_hpp

//std
#include <memory>

namespace romea {


class OneAxleSteeringConstraints
{

public:

  OneAxleSteeringConstraints();

  OneAxleSteeringConstraints(const double & minimalLinearSpeed,
                             const double & maximalLinearSpeed,
                             const double & maximalAbsoluteSteeringAngle);

public :

  void setMinimalLinearSpeed(const double & minimalLinearSpeed);
  void setMaximalLinearSpeed(const double & maximalLinearSpeed);
  void setMaximalAbsoluteSteeringAngle(const double & maximalAbsoluteSteeringAngle);

  const double & getMinimalLinearSpeed() const;
  const double & getMaximalLinearSpeed() const;
  const double & getMaximalAbsoluteSteeringAngle() const;

protected :

  double minimalLinearSpeed_;
  double maximalLinearSpeed_;
  double maximalAbsoluteSteeringAngle_;
};


}//end romea
#endif
