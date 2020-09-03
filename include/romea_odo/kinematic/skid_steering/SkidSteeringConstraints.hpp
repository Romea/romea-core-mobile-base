#ifndef romea_SkidSteeringConstraints_hpp
#define romea_SkidSteeringConstraints_hpp

//std
#include <memory>

namespace romea {


class SkidSteeringConstraints
{

public:

  SkidSteeringConstraints();

  SkidSteeringConstraints(const double & minimalLinearSpeed,
                          const double &maximalLinearSpeed,
                          const double & maximalAbsoluteAngularSpeed);

public :

  void setMinimalLinearSpeed(const double & minimalLinearSpeed);
  void setMaximalLinearSpeed(const double & maximalLinearSpeed);
  void setMaximalAbsoluteAngularSpeed(const double & maximalAbsoluteAngularSpeed);

  const double & getMinimalLinearSpeed() const;
  const double & getMaximalLinearSpeed() const;
  const double & getMaximalAbsoluteAngularSpeed() const;

protected :

  double minimalLinearSpeed_;
  double maximalLinearSpeed_;
  double maximalAbsoluteAngularSpeed_;
};


}//end romea
#endif
