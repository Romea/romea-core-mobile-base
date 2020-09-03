#include "romea_odo/kinematic/skid_steering/SkidSteeringMeasure.hpp"
#include <romea_common/math/Algorithm.hpp>

namespace {
const double EPSILON = 0.0000001;
}

namespace romea {

//-----------------------------------------------------------------------------
SkidSteeringMeasure::SkidSteeringMeasure():
  covariance(Eigen::Matrix2d::Zero())
{

}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const SkidSteeringMeasure & measure)
{
  KinematicMeasure convertedMeasure;
  convertedMeasure.longitudinalSpeed=measure.longitudinalSpeed;
  convertedMeasure.angularSpeed = measure.angularSpeed;

  if(std::abs(measure.angularSpeed)<EPSILON)
  {
    convertedMeasure.instantaneousCurvature=0;
  }
  else if (std::abs(measure.longitudinalSpeed)>EPSILON)
  {
    convertedMeasure.instantaneousCurvature=measure.angularSpeed/measure.longitudinalSpeed;
  }
  else
  {
    convertedMeasure.instantaneousCurvature = romea::sign(measure.angularSpeed)*std::numeric_limits<double>::max();
  }


  Eigen::MatrixXd J =  Eigen::MatrixXd::Zero(4,2);
  J(0,0)=1;
  J(2,1)=1;
  J(3,1)=measure.angularSpeed;

  if(std::abs(measure.longitudinalSpeed)<EPSILON)
  {
    J(3,0)=-measure.angularSpeed/(std::pow(romea::sign(measure.longitudinalSpeed)*EPSILON,2));
  }

  convertedMeasure.covariance = J*measure.covariance*J.transpose();
  return convertedMeasure;
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const SkidSteeringMeasure & command,
                                    const SkidSteeringKinematic::Parameters & /*parameters*/)
{
  return toKinematicMeasure(command);
}

////-----------------------------------------------------------------------------
//SkidSteeringMeasure toSkidSteeringMeasure(const KinematicMeasure & measure)
//{
//  assert(measure.beta<std::numeric_limits<double>::epsilon());
//  SkidSteeringMeasure convertedMeasure;
//  convertedMeasure.longitudinalSpeed=measure.speed;
//  convertedMeasure.angularSpeed=measure.angularSpeed;
//  convertedMeasure.covariance(0,0) = measure.covariance(0,0);
//  convertedMeasure.covariance(0,1) = measure.covariance(0,2);
//  convertedMeasure.covariance(1,0) = measure.covariance(2,0);
//  convertedMeasure.covariance(1,1) = measure.covariance(2,2);
//  return convertedMeasure;
//}

////-----------------------------------------------------------------------------
//SkidSteeringMeasure toSkidSteeringCommand(const KinematicMeasure & measure,const Kinematic & /*kinematic*/)
//{
//  return toSkidSteeringMeasure(measure);
//}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const SkidSteeringMeasure & measure)
{
  os<<" SkidSteering measure   "<<std::endl;;
  os<<" measured linear speed  " << measure.longitudinalSpeed << std::endl;
  os<<" measured angular speed " << measure.angularSpeed << std::endl;
  os<<" measured covariance matrix " << std::endl;
  os<< measure.covariance;

  return os;
}

}//end romea

