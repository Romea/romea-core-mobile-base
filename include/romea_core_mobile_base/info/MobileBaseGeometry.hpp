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

struct ContinuousTrack
{
  ContinuousTrack();
  double width;
  //TODO finish
};

struct WheeledAxle
{
  WheeledAxle();
  double wheelsDistance;
  Wheel wheels;
};

struct ContinuousTrackedAxle
{
  ContinuousTrackedAxle();
  double tracksDistance;
  ContinuousTrack tracks;
};


template<typename FrontAxle, typename RearAxle>
struct TwoAxles
{
  TwoAxles();
  double axlesDistance;
  FrontAxle frontAxle;
  RearAxle rearAxle;
};

using TwoWheeledAxles = TwoAxles<WheeledAxle,WheeledAxle>;
using TwoContinousTrackedAxles =  TwoAxles<ContinuousTrack,ContinuousTrack>;


//std::ostream& operator<<(std::ostream& os, const Wheel & wheel);

//template<typename Wheel>
//std::ostream& operator<<(std::ostream& os, const OneAxleGeometry<Wheel> & oneAxleGeometry);

//template<typename FrontWheel,typename RearWheel>
//std::ostream& operator<<(std::ostream& os, const TwoAxlesGeometry<FrontWheel,RearWheel> & twoAxlesGeometry);

}

#endif
