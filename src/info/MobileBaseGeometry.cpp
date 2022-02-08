//romea
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"

//std
#include <limits>

namespace romea
{

//-----------------------------------------------------------------------------
Wheel::Wheel():
  hubCarrierOffset(std::numeric_limits<double>::quiet_NaN()),
  radius(std::numeric_limits<double>::quiet_NaN()),
  width(std::numeric_limits<double>::quiet_NaN())
{

}

////-----------------------------------------------------------------------------
//MecanumWheel::MecanumWheel():
//  RubberWheel(),
//  numberOfRollers(0)
//{

//}

//-----------------------------------------------------------------------------
ContinuousTrack::ContinuousTrack():
  width(std::numeric_limits<double>::quiet_NaN())
{

}

//-----------------------------------------------------------------------------
template<typename Wheel>
OneAxleGeometry<Wheel>::OneAxleGeometry():
  wheelTrack(std::numeric_limits<double>::quiet_NaN()),
  wheels()
{
};

//-----------------------------------------------------------------------------
template<typename FrontWheel, typename RearWheel>
TwoAxlesGeometry<FrontWheel,RearWheel>::TwoAxlesGeometry():
  wheelbase(std::numeric_limits<double>::quiet_NaN()),
  frontAxle(),
  rearAxle()
{
};

//std::ostream& operator<<(std::ostream& os, const RubberWheel & wheel);
//std::ostream& operator<<(std::ostream& os, const OneAxleGeometry<RubberWheel> & oneAxleGeometry);
//std::ostream& operator<<(std::ostream& os, const TwoAxleGeometry<RubberWheel> & twoAxlesGeometry);


template struct OneAxleGeometry<Wheel>;
template struct OneAxleGeometry<ContinuousTrack>;
template struct TwoAxlesGeometry<Wheel,Wheel>;
template struct TwoAxlesGeometry<Wheel,ContinuousTrack>;
template struct TwoAxlesGeometry<ContinuousTrack,ContinuousTrack>;

}
