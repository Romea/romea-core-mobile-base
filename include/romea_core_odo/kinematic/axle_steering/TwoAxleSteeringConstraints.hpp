#ifndef romea_TwoAxleSteeringConstraints_hpp
#define romea_TwoAxleSteeringConstraints_hpp

//std
#include <memory>

namespace romea {


class TwoAxleSteeringConstraints
{

public:

  TwoAxleSteeringConstraints();

  TwoAxleSteeringConstraints(const double & minimalLinearSpeed,
                             const double & maximalLinearSpeed,
                             const double & maximalAbsoluteFrontSteeringAngle,
                             const double & maximalAbsoluteRearSteeringAngle);

public :

  void setMinimalLinearSpeed(const double & minimalLinearSpeed);
  void setMaximalLinearSpeed(const double & maximalLinearSpeed);
  void setMaximalAbsoluteFrontSteeringAngle(const double & maximalAbsoluteFrontSteeringAngle);
  void setMaximalAbsoluteRearSteeringAngle(const double & maximalAbsoluteRearSteeringAngle);

  const double & getMinimalLinearSpeed() const;
  const double & getMaximalLinearSpeed() const;
  const double & getMaximalAbsoluteFrontSteeringAngle() const;
  const double & getMaximalAbsoluteRearSteeringAngle() const;

protected :

  double minimalLinearSpeed_;
  double maximalLinearSpeed_;
  double maximalAbsoluteFrontSteeringAngle_;
  double maximalAbsoluteRearSteeringAngle_;

};


}//end romea
#endif
