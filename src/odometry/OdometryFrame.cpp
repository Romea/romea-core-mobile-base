#include "romea_odo/odometry/OdometryFrame.hpp"
#include <iostream>
namespace romea {


////-----------------------------------------------------------------------------
//OdometryFrame::OdometryFrame():
//  data_()
//{

//}

////-----------------------------------------------------------------------------
//void OdometryFrame::set(const std::string & name, const double &value)
//{
//  data_[name]=value;
//}

////-----------------------------------------------------------------------------
//bool OdometryFrame::get(const std::string & name, double & value) const
//{
//  auto it = data_.find(name);
//  if(it==data_.end())
//  {
//    return false;
//  }
//  value=it->second;
//  return true;
//}

////-----------------------------------------------------------------------------
//std::vector<std::string> OdometryFrame::getNames()const
//{
//   std::vector<std::string> names;
//   names.reserve(data_.size());
//   for (const auto & [name,value] : data_)
//   {
//       names.push_back(name);
//   }
//   return names;
//}

////-----------------------------------------------------------------------------
//std::vector<double> OdometryFrame::getValue()const
//{
//    std::vector<double> values;
//    values.reserve(data_.size());
//    for (const auto & [name,value] : data_)
//    {
//        values.push_back(value);
//    }
//    return values;
//}


//bool setMeasurement(const OdometryFrame & frame, const std::string & name, const double & value)
//{
//    auto it = std::find(frame.measurement_names.begin(),
//                        frame.measurement_names.end(),
//                        name);

//    if(it!=frame.measurement_names.end())
//    {
//        return false;
//    }
//    else
//    {
//      frame.measurement_names.push_back(name)
//      frame.measurements.push_back(value);
//      return true;
//    }
//}


//bool getMeasurement(const OdometryFrame & frame, const std::string & name, double & value)
//{

//    auto it = std::find(frame.begin(),
//                        frame.end(),
//                             name);


//    if(it==frame.measurement_names.end())
//    {
//        return false;
//    }
//    else
//    {
//      value= frame.measurements[std::distance(frame.measurement_names.begin(),it)];
//      return true;
//    }

//}


////-----------------------------------------------------------------------------
//std::ostream & operator<<(std::ostream &s, const OdometryFrame &frame)
//{
//  s << "speeds ";
//  for(const auto & speed: frame.getWheelSpeeds())
//  {
//    s << speed << " ";
//  }
//  s<< std::endl;

//  s << "steerings ";
//  for(const auto & steering: frame.getSteeringAngles())
//  {
//    s << steering << " ";
//  }
//  s << std::endl;
//  return s;
//}



}
