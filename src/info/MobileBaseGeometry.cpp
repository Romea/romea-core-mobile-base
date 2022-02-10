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

//-----------------------------------------------------------------------------
ContinuousTrack::ContinuousTrack():
  width(std::numeric_limits<double>::quiet_NaN())
{

}

//-----------------------------------------------------------------------------
WheeledAxle::WheeledAxle():
  wheelsDistance(std::numeric_limits<double>::quiet_NaN()),
  wheels()
{
};

//-----------------------------------------------------------------------------
ContinuousTrackedAxle::ContinuousTrackedAxle():
  tracksDistance(std::numeric_limits<double>::quiet_NaN()),
  tracks()
{
};

//-----------------------------------------------------------------------------
template<typename FrontAxle, typename RearAxle>
TwoAxles<FrontAxle,RearAxle>::TwoAxles():
  axlesDistance(std::numeric_limits<double>::quiet_NaN()),
  frontAxle(),
  rearAxle()
{
};

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const Wheel & wheel)
//{
//  os << " radius: " << wheel.radius <<"m"<< std::endl;
//  os << " width: " << wheel.width <<"m"<< std::endl;
//  os << " hub_carrier_offset: " << wheel.hubCarrierOffset <<"m"<<std::endl;
//  return os;
//}


////-----------------------------------------------------------------------------
//template<typename Wheel>
//std::ostream& operator<<(std::ostream& os, const OneAxleGeometry<Wheel> & oneAxleGeometry)
//{
//  os << " wheel_track " << oneAxleGeometry.wheelTrack<<std::endl;
//  os << " wheels : "<<std::endl;
//  os << oneAxleGeometry.wheels;
//}

////std::ostream& operator<<(std::ostream& os, const TwoAxleGeometry<RubberWheel> & twoAxlesGeometry);


template struct TwoAxles<WheeledAxle,WheeledAxle>;
template struct TwoAxles<WheeledAxle,ContinuousTrackedAxle>;
template struct TwoAxles<ContinuousTrackedAxle,ContinuousTrackedAxle>;

}
