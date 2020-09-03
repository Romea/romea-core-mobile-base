#include "romea_odo/odometry/OdometryFrame.hpp"
#include <iostream>
namespace romea {


//-----------------------------------------------------------------------------
OdometryFrame::OdometryFrame():
  wheelSpeeds_(),
  steeringAngles_()
{

}

//-----------------------------------------------------------------------------
void OdometryFrame::setWheelSpeed(const std::string & wheelName, const double &speed)
{
  wheelSpeeds_[wheelName]=speed;
}

//-----------------------------------------------------------------------------
bool OdometryFrame::getWheelSpeed(const std::string & wheelName, double & speed) const
{
  auto it = wheelSpeeds_.find(wheelName);
  if(it==wheelSpeeds_.end())
  {
    return false;
  }
  speed=it->second;
  return true;
}

//-----------------------------------------------------------------------------
void OdometryFrame::setSteeringAngle(const std::string & steeringName, const double &angle)
{
  steeringAngles_[steeringName]=angle;
}

//-----------------------------------------------------------------------------
bool OdometryFrame::getSteeringAngle(const std::string & steeringName, double & angle)const
{
  auto it = steeringAngles_.find(steeringName);  
  if(it==steeringAngles_.end())
  {
    return false;
  }
  angle=it->second;
  return true;

}

//-----------------------------------------------------------------------------
std::vector<double>  OdometryFrame::getWheelSpeeds() const
{
  std::vector<double> wheelSpeeds;
  wheelSpeeds.reserve(wheelSpeeds_.size());
  for( auto it : wheelSpeeds_ )
  {
    wheelSpeeds.push_back(it.second);
  }
  return wheelSpeeds;
}

//-----------------------------------------------------------------------------
std::vector<double>  OdometryFrame::getSteeringAngles() const
{
  std::vector<double> steeringAngles;
  steeringAngles.reserve(steeringAngles_.size());
  for( auto it : steeringAngles_ )
  {
    steeringAngles.push_back(it.second);
  }
  return steeringAngles;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &s, const OdometryFrame &frame)
{
  s << "speeds ";
  for(const auto & speed: frame.getWheelSpeeds())
  {
    s << speed << " ";
  }
  s<< std::endl;

  s << "steerings ";
  for(const auto & steering: frame.getSteeringAngles())
  {
    s << steering << " ";
  }
  s << std::endl;
  return s;
}



}
