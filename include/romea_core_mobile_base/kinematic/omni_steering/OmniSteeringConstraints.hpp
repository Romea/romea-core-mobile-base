#ifndef romea_OmniSteeringConstraints_hpp
#define romea_OmniSteeringConstraints_hpp

//std
#include <memory>


namespace romea {


class OmniSteeringConstraints
{
  
public:
  
  OmniSteeringConstraints();
  
  OmniSteeringConstraints(const double & minimalLongitudinalSpeed,
                          const double & maximalLongitudinalSpeed,
                          const double & maximalAbsoluteLateralSpeed,
                          const double & maximalAbsoluteAngularSpeed);
  
public :
  
  void setMinimalLongitudinalSpeed(const double & minimalLongitudinalSpeed);
  void setMaximalLongitudinalSpeed(const double & maximalLongitudinalSpeed);
  void setMinimalAbsoluteLateralSpeed(const double & maximalAbsoluteLateralSpeed);
  void setMaximalAbsoluteAngularSpeed(const double & maximalAbsoluteAngularSpeed);
  
  const double & getMinimalLongitudinalSpeed() const;
  const double & getMaximalLongitudinalSpeed() const;
  const double & getMaximalAbsoluteLateralSpeed() const;
  const double & getMaximalAbsoluteAngularSpeed() const;
  
protected :
  
  double minimalLongitudinalSpeed_;
  double maximalLongitudinalSpeed_;
  double maximalAbsoluteLateralSpeed_;
  double maximalAbsoluteAngularSpeed_;
};


}//end romea
#endif
