#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringMeasure.hpp"
//#include <math/Algorithm.hpp>

namespace romea {

//-----------------------------------------------------------------------------
OmniSteeringMeasure::OmniSteeringMeasure():
  covariance(Eigen::Matrix3d::Zero())
{

}


//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const OmniSteeringMeasure & measure)
{

  KinematicMeasure convertedMeasure;
  convertedMeasure.longitudinalSpeed=measure.longitudinalSpeed;
  convertedMeasure.lateralSpeed=measure.lateralSpeed;
  convertedMeasure.angularSpeed =measure.angularSpeed;

//  if(instantaneousCurvature)
//  {
//    convertedMeasure.instantaneousCurvature = *instantaneousCurvature;
//  }
//  else
//  {
//    convertedMeasure.instantaneousCurvature = 0;
//  }


//  Eigen::MatrixXd J =  Eigen::MatrixXd::Identity(4,3);s
//  J.row(0,0;
//  J.row(1) << measure.beta/alpha,measure.speed/alpha,0;
//  J.row(2) << 0 , 0 , 1 ;
//  J.row(3) << angularSpeed*measure.speed/gamma , angularSpeed*measure.beta/gamma, 1/speed;


//  convertedMeasure.covariance = J*measure.covariance*J.transpose();

  return convertedMeasure;
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const OmniSteeringMeasure & measure, const  MecanumWheelSteeringKinematic::Parameters & /*parameters*/)
{
  return toKinematicMeasure(measure);
}


//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const OmniSteeringMeasure & measure)
{
  os<<" OmniSteering measure   "<<std::endl;;
  os<<" measured longitudinal speed  " << measure.longitudinalSpeed<< std::endl;
  os<<" measured latreal speed  " << measure.lateralSpeed << std::endl;
  os<<" measured angular speed " << measure.angularSpeed << std::endl;
  os<<" measured covariance matrix " << std::endl;
  os<< measure.covariance;

  return os;
}

}//end romea

