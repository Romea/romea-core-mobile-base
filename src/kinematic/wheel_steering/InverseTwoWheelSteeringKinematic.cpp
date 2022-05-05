//romea
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include <iostream>

namespace romea {


//-----------------------------------------------------------------------------
void inverseKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2FWS2FWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{

  const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
  const double &wheelAngleVariance = parameters.wheelAngleVariance;

  const double halfTrack = parameters.frontWheelTrack/2;
  const double wheelBase = parameters.frontWheelBase+parameters.rearWheelBase;
  const double hubCarrierOffset = parameters.frontHubCarrierOffset;

  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
  double sinLeft = std::sin(frontLeftWheelAngle);
  double cosLeft = std::cos(frontLeftWheelAngle);
  double alphaLeft = cosLeft + sinLeft*halfTrack/wheelBase;
  double betaLeft = 1- hubCarrierOffset*sinLeft/wheelBase;

  const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
  double sinRight = std::sin(frontRightWheelAngle);
  double cosRight = std::cos(frontRightWheelAngle);
  double alphaRight = cosRight - sinRight*halfTrack/wheelBase;
  double betaRight = 1+ hubCarrierOffset*sinRight/wheelBase;

  double gamma = 1/ (1 + std::pow(0.5*(sinLeft/alphaLeft + sinRight/alphaRight),2));

  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
  covariance(0,0)=wheelSpeedVariance;
  covariance(1,1)=wheelSpeedVariance;
  covariance(2,2)=wheelAngleVariance;
  covariance(3,3)=wheelAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2,4);
  J(0,0) = 0.5*alphaLeft/betaLeft;
  J(0,1) = 0.5*alphaRight/betaRight;
  J(0,2) = frontLeftWheelSpeed*((-sinLeft+cosLeft*halfTrack/wheelBase) - alphaLeft*hubCarrierOffset*cosLeft/wheelBase)/(betaLeft*betaLeft);
  J(0,3) = frontRightWheelSpeed*((-sinRight-cosRight*halfTrack/wheelBase) - alphaRight*hubCarrierOffset*cosRight/wheelBase)/(betaRight*betaRight);
  J(1,2) = gamma/(alphaLeft*alphaLeft);
  J(1,3) = gamma/(alphaRight*alphaRight);


  oneAxleSteeringMeasure.steeringAngle= std::atan(0.5*(sinLeft/alphaLeft + sinRight/alphaRight));
  oneAxleSteeringMeasure.longitudinalSpeed = 0.5*(frontLeftWheelSpeed*alphaLeft/betaLeft+ frontRightWheelSpeed*alphaRight/betaRight);
  oneAxleSteeringMeasure.covariance =  J*covariance*J.transpose();
}

//-----------------------------------------------------------------------------
void inverseKinematic(const TwoWheelSteeringKinematic::Parameters &parameters,
                      const OdometryFrame2FWS2RWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{

  const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
  const double &wheelAngleVariance = parameters.wheelAngleVariance;

  const double wheelBase = parameters.frontWheelBase+parameters.rearWheelBase;
  const double frontHalfTrack = parameters.frontWheelTrack/2;
  const double rearHalfTrack = parameters.rearWheelTrack/2 + parameters.rearHubCarrierOffset;


  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;
  double sinLeft = std::sin(frontLeftWheelAngle);
  double cosLeft = std::cos(frontLeftWheelAngle);
  double alphaLeft = cosLeft + sinLeft*frontHalfTrack/wheelBase;
  double betaLeft = cosLeft + sinLeft*(frontHalfTrack-rearHalfTrack)/wheelBase;
  double deltaLeft = 1 + rearHalfTrack *sinLeft/betaLeft;

  const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;
  double sinRight = std::sin(frontRightWheelAngle);
  double cosRight = std::cos(frontRightWheelAngle);
  double alphaRight = cosRight - sinRight*frontHalfTrack/wheelBase;
  double betaRight = cosRight - sinRight*(frontHalfTrack-rearHalfTrack)/wheelBase;
  double deltaRight = 1 - rearHalfTrack*sinRight/betaRight;

  double gamma = 1/ (1 + std::pow(0.5*(sinLeft/alphaLeft + sinRight/alphaRight),2));

  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
  covariance(0,0)=wheelSpeedVariance;
  covariance(1,1)=wheelSpeedVariance;
  covariance(2,2)=wheelAngleVariance;
  covariance(3,3)=wheelAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2,4);
  J(0,0) = 0.5*deltaLeft;
  J(0,1) = 0.5*deltaRight;
  J(0,2) = rearLeftWheelSpeed*rearHalfTrack/(deltaLeft*deltaLeft);
  J(0,3) = rearRightWheelSpeed*rearHalfTrack/(deltaRight*deltaRight);
  J(1,2) = gamma/(alphaLeft*alphaLeft);
  J(1,3) = gamma/(alphaLeft*alphaLeft);

  oneAxleSteeringMeasure.steeringAngle= std::atan(0.5*(sinLeft/alphaLeft + sinRight/alphaRight));
  oneAxleSteeringMeasure.longitudinalSpeed = 0.5*(rearLeftWheelSpeed*deltaLeft+rearRightWheelSpeed*deltaRight);
  oneAxleSteeringMeasure.covariance =  J*covariance*J.transpose();
}

//-----------------------------------------------------------------------------
void inverseKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2FWS4WD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
    const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
    const double &wheelAngleVariance = parameters.wheelAngleVariance;

    const double wheelBase = parameters.frontWheelBase+parameters.rearWheelBase;
    const double fronthubCarrierOffset = parameters.frontHubCarrierOffset;
    const double rearhubCarrierOffset = parameters.rearHubCarrierOffset;
    const double frontHalfTrack = parameters.frontWheelTrack/2;
    const double rearHalfTrack = parameters.rearWheelTrack/2 +rearhubCarrierOffset;

    const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
    const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
    const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;
    double sinLeft = std::sin(frontLeftWheelAngle);
    double cosLeft = std::cos(frontLeftWheelAngle);
    double alphaLeft = cosLeft + sinLeft*frontHalfTrack/wheelBase;
    double betaLeft = cosLeft + sinLeft*(frontHalfTrack-rearHalfTrack)/wheelBase;
    double gammaLeft = 1 - fronthubCarrierOffset*sinLeft/wheelBase;
    double deltaLeft = 1 + rearHalfTrack *sinLeft/betaLeft;

    const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
    const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
    const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;
    double sinRight = std::sin(frontRightWheelAngle);
    double cosRight = std::cos(frontRightWheelAngle);
    double alphaRight = cosRight - sinRight*frontHalfTrack/wheelBase;
    double betaRight = cosRight - sinRight*(frontHalfTrack-rearHalfTrack)/wheelBase;
    double gammaRight = 1 + fronthubCarrierOffset*sinRight/wheelBase;
    double deltaRight = 1 - rearHalfTrack*sinRight/betaRight;

    double epsilon = 1/ (1 + std::pow(0.5*(sinLeft/alphaLeft + sinRight/alphaRight),2));

    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(6,6);
    covariance(0,0)=wheelSpeedVariance;
    covariance(1,1)=wheelSpeedVariance;
    covariance(2,2)=wheelSpeedVariance;
    covariance(3,3)=wheelSpeedVariance;
    covariance(4,4)=wheelAngleVariance;
    covariance(5,5)=wheelAngleVariance;

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2,6);

    J(0,0) = 0.5*alphaLeft/betaLeft;
    J(0,1) = 0.5*alphaRight/betaRight;
    J(0,2) = 0.5*deltaLeft;
    J(0,3) = 0.5*deltaRight;
    J(0,4) += frontLeftWheelSpeed*((-sinLeft+cosLeft*frontHalfTrack/wheelBase) - alphaLeft*fronthubCarrierOffset*cosLeft/wheelBase)/(gammaLeft*gammaLeft);
    J(0,4) += rearLeftWheelSpeed*rearHalfTrack/(deltaLeft*deltaLeft);
    J(0,5) += frontRightWheelSpeed*((-sinRight-cosRight*frontHalfTrack/wheelBase) - alphaRight*fronthubCarrierOffset*cosRight/wheelBase)/(gammaRight*gammaRight);
    J(0,5) += rearRightWheelSpeed*rearHalfTrack/(deltaRight*deltaRight);
    J(1,4) = epsilon/(alphaLeft*alphaLeft);
    J(1,5) = epsilon/(alphaLeft*alphaLeft);

    oneAxleSteeringMeasure.steeringAngle= std::atan(0.5*(sinLeft/alphaLeft + sinRight/alphaRight));
    oneAxleSteeringMeasure.longitudinalSpeed = 0.25*(rearLeftWheelSpeed*deltaLeft+rearRightWheelSpeed*deltaRight) +
            0.25*(frontLeftWheelSpeed*alphaLeft/gammaLeft+ frontRightWheelSpeed*alphaRight/gammaRight);

}

}
