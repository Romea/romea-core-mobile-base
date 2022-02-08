#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea {

//-----------------------------------------------------------------------------
OneAxleSteeringMeasure::OneAxleSteeringMeasure():
  covariance(Eigen::Matrix2d::Zero())
{

}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
                                    const double & frontWheelBase,
                                    const double & rearWheelBase)
{
  double wheelbase = frontWheelBase+rearWheelBase;
  double tanSteeringAngle = std::tan(measure.steeringAngle);
  double instantaneousCurvature = tanSteeringAngle/wheelbase;
  double tanBeta = instantaneousCurvature*rearWheelBase;
  double alpha = (1 + tanSteeringAngle*tanSteeringAngle)/wheelbase;
  double squareGamma = (1 + tanBeta* tanBeta);
  double gamma = std::sqrt(squareGamma);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4,2);
  J(0,0)= 1;
  J(1,0)= tanBeta;
  J(1,1)= measure.longitudinalSpeed*alpha*rearWheelBase;
  J(2,0)= instantaneousCurvature ;
  J(2,1)= measure.longitudinalSpeed*alpha;
  J(3,1)= alpha/squareGamma*(gamma-instantaneousCurvature*rearWheelBase/gamma);

  KinematicMeasure convertedMeasure;
  convertedMeasure.longitudinalSpeed=measure.longitudinalSpeed;
  convertedMeasure.lateralSpeed=measure.longitudinalSpeed*tanBeta;
  convertedMeasure.angularSpeed =instantaneousCurvature*measure.longitudinalSpeed;
  convertedMeasure.instantaneousCurvature =instantaneousCurvature/gamma;
  convertedMeasure.covariance =J*measure.covariance *J.transpose();
  return convertedMeasure;
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
                                    const OneAxleSteeringKinematic::Parameters & parameters)
{
  return toKinematicMeasure(measure,parameters.frontWheelBase,parameters.rearWheelBase);
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
                                    const TwoWheelSteeringKinematic::Parameters & parameters)
{
  return toKinematicMeasure(measure,parameters.frontWheelBase,parameters.rearWheelBase);
}


////-----------------------------------------------------------------------------
//KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
//                                    const double & wheelBase)
//{

//  double tanSteeringAngle = std::tan(measure.steeringAngle);
//  double beta = (1 + tanSteeringAngle*tanSteeringAngle)/wheelBase;
//  double instantaneousCurvature =tanSteeringAngle/wheelBase;


//  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4,2);
//  J(0,0)=1;
//  J(2,0)=instantaneousCurvature;
//  J(2,1)=measure.longitudinalSpeed*beta;
//  J(3,1)=beta;

//  KinematicMeasure convertedMeasure;
//  convertedMeasure.speed = measure.longitudinalSpeed;
//  convertedMeasure.angularSpeed = instantaneousCurvature*measure.longitudinalSpeed;
//  convertedMeasure.instantaneousCurvature = instantaneousCurvature;
//  convertedMeasure.covariance =J*measure.covariance*J.transpose();
//  return convertedMeasure;
//}


////-----------------------------------------------------------------------------
//KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
//                                    const Kinematic & kinematic)
//{
//  return toKinematicMeasure(measure,kinematic.getWheelBase("wheelbase"));
//}


////-----------------------------------------------------------------------------
//KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
//                                    const double & wheelBase)
//{

//  double tanSteeringAngle = std::tan(measure.steeringAngle);
//  double beta = (1 + tanSteeringAngle*tanSteeringAngle)/wheelBase;
//  double instantaneousCurvature =tanSteeringAngle/wheelBase;


//  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4,2);
//  J(0,0)=1;
//  J(2,0)=instantaneousCurvature;
//  J(2,1)=measure.longitudinalSpeed*beta;
//  J(3,1)=beta;

//  KinematicMeasure convertedMeasure;
//  convertedMeasure.speed = measure.longitudinalSpeed;
//  convertedMeasure.angularSpeed = instantaneousCurvature*measure.longitudinalSpeed;
//  convertedMeasure.instantaneousCurvature = instantaneousCurvature;
//  convertedMeasure.covariance =J*measure.covariance*J.transpose();
//  return convertedMeasure;
//}


////-----------------------------------------------------------------------------
//KinematicMeasure toKinematicMeasure(const OneAxleSteeringMeasure & measure,
//                                    const Kinematic & kinematic)
//{
//  return toKinematicMeasure(measure,kinematic.getWheelBase("wheelbase"));
//}

////-----------------------------------------------------------------------------
//OneAxleSteeringMeasure toOneAxleSteeringMeasure(const KinematicMeasure & measure,
//                                                const double &wheelBase)
//{
//  assert(measure.beta<std::numeric_limits<double>::epsilon());

//  double alpha = measure.instantaneousCurvature*wheelBase;
//  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2,4);
//  J(0,0)=1;
//  J(1,3)= wheelBase/(alpha*alpha+1);

//  OneAxleSteeringMeasure convertedMeasure;
//  convertedMeasure.longitudinalSpeed =measure.speed;
//  convertedMeasure.steeringAngle=std::atan(alpha);
//  convertedMeasure.covariance = J*measure.covariance*J.transpose();
//  return convertedMeasure;
//}

////-----------------------------------------------------------------------------
//OneAxleSteeringMeasure toOneAxleSteeringMeasure(const KinematicMeasure & measure,
//                                                const Kinematic & kinematic)
//{
//  return toOneAxleSteeringMeasure(measure,kinematic.getWheelBase("wheelbase"));
//}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const OneAxleSteeringMeasure & measure)
{
  os<<" OneAxleSteeringMeasure measure   "<<std::endl;;
  os<<" measured linear speed  " << measure.longitudinalSpeed << std::endl;
  os<<" measured front steering angle " << measure.steeringAngle << std::endl;
  os<<" measured covariance matrix " << std::endl;
  os<< measure.covariance;

  return os;
}


}//end romea

