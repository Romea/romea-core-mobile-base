#ifndef romea_MobileBaseGeometry_hpp
#define romea_MobileBaseGeometry_hpp

#include <iostream>
#include <vector>
#include <array>

namespace romea
{

struct Wheel
{
  double radius;
  double width;
  double hubCarrierOffset;
};

struct TrackWheel
{
  double radius;
  double x;
  double z;
};

struct ContinuousTrack
{
 double width;
 double thickness;
 TrackWheel sprocketWheel;
 std::vector<TrackWheel> idlerWheels;
 std::vector<TrackWheel> rollerWheels;
};

struct WheeledAxle
{
  double wheelsDistance;
  Wheel wheels;
};

struct ContinuousTrackedAxle
{
  double tracksDistance;
  ContinuousTrack tracks;
};


template<typename FrontAxle, typename RearAxle>
struct TwoAxles
{
  double axlesDistance;
  FrontAxle frontAxle;
  RearAxle rearAxle;
};

using TwoWheeledAxles = TwoAxles<WheeledAxle,WheeledAxle>;




//std::ostream& operator<<(std::ostream& os, const Wheel & wheel);

//template<typename Wheel>
//std::ostream& operator<<(std::ostream& os, const OneAxleGeometry<Wheel> & oneAxleGeometry);

//template<typename FrontWheel,typename RearWheel>
//std::ostream& operator<<(std::ostream& os, const TwoAxlesGeometry<FrontWheel,RearWheel> & twoAxlesGeometry);

}

#endif
