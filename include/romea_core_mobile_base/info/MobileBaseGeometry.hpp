#ifndef romea_MobileBaseGeometry_hpp
#define romea_MobileBaseGeometry_hpp

#include <iostream>

namespace romea
{

struct Wheel
{
  Wheel();
  double hubCarrierOffset;
  double radius;
  double width;
};

//struct MecanumWheel : RubberWheel
//{
//  MecanumWheel();
//  size_t numberOfRollers;
//  double rollersRadius;
//  double rollersLength;
//};

struct ContinuousTrack
{
  ContinuousTrack();
  double width;
  //TODO finish
};

template<typename Wheel>
struct OneAxleGeometry
{
  OneAxleGeometry();
  double wheelTrack;
  Wheel wheels;
};

template<typename FrontWheel, typename RearWheel>
struct TwoAxlesGeometry
{
  TwoAxlesGeometry();
  double wheelbase;
  OneAxleGeometry<FrontWheel> frontAxle;
  OneAxleGeometry<RearWheel> rearAxle;
};

std::ostream& operator<<(std::ostream& os, const Wheel & wheel);

template<typename Wheel>
std::ostream& operator<<(std::ostream& os, const OneAxleGeometry<Wheel> & oneAxleGeometry);

template<typename FrontWheel,typename RearWheel>
std::ostream& operator<<(std::ostream& os, const TwoAxlesGeometry<FrontWheel,RearWheel> & twoAxlesGeometry);

}

#endif
