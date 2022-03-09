#ifndef romea_MobileBaseGeometry_hpp
#define romea_MobileBaseGeometry_hpp

#include <iostream>

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
};

struct FlatContinuousTrack
{
  double width;
  TrackWheel sprocket_wheel;
  TrackWheel idler_wheel;
};

struct TriangularContinuousTrack
{
  double width;
  TrackWheel high_sprocket_wheel;
  TrackWheel front_idler_wheel;
  TrackWheel rear_idler_wheel;
};

struct WheeledAxle
{
  double wheelsDistance;
  Wheel wheels;
};


template <typename ContinuousTrack>
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

using FlatContinuousTrackedAxle = ContinuousTrackedAxle<FlatContinuousTrack>;
using TriangularContinuousTrackedAxle = ContinuousTrackedAxle<TriangularContinuousTrack>;



//std::ostream& operator<<(std::ostream& os, const Wheel & wheel);

//template<typename Wheel>
//std::ostream& operator<<(std::ostream& os, const OneAxleGeometry<Wheel> & oneAxleGeometry);

//template<typename FrontWheel,typename RearWheel>
//std::ostream& operator<<(std::ostream& os, const TwoAxlesGeometry<FrontWheel,RearWheel> & twoAxlesGeometry);

}

#endif
