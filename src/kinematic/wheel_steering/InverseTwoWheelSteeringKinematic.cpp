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


////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic2FWS2FWD(const TwoWheelSteeringKinematic & kinematic,
//                                          const OdometryFrame2FWS2FWD & odometryFrame,
//                                          const double & wheelSpeedVariance,
//                                          const double & wheelAngleVariance)
//{

//  const double & hubCarrierOffset = kinematic.getHubCarrierOffset("front_hub_carrier_offset");
//  const double wheelBase= kinematic.getWheelBase("wheelbase");
//  const double track  = kinematic.getTrack("front_track");

//  //Left information
//  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
//  double leftInstantaneousCurvature = OneAxleSteeringKinematic::computeInstantaneousCurvature(frontLeftWheelAngle,wheelBase);

//  double rearLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed*std::cos(frontLeftWheelAngle)/
//      (1-hubCarrierOffset*leftInstantaneousCurvature*std::cos(frontLeftWheelAngle));


//  //Right information
//  const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
//  double rightInstantaneousCurvature = OneAxleSteeringKinematic::computeInstantaneousCurvature(frontRightWheelAngle,wheelBase);

//  double rearRightWheelSpeed = odometryFrame.frontRightWheelSpeed*std::cos(frontRightWheelAngle)/
//      (1+hubCarrierOffset*rightInstantaneousCurvature*std::cos(frontRightWheelAngle));

//  //Compute kinematic values
//  double linearSpeed = 0.5*(rearLeftWheelSpeed+rearRightWheelSpeed);
//  double angularSpeed = 0.5 * (leftInstantaneousCurvature*rearLeftWheelSpeed +rightInstantaneousCurvature*rearRightWheelSpeed);
//  double instantaneousCurvature = TwoWheelSteeringKinematic::computeInstantaneousCurvature(leftInstantaneousCurvature,rightInstantaneousCurvature,track);

//  //Compute covariance matrix

//  double halfTrack = track/2;
//  double sinLeft = std::sin(frontLeftWheelAngle);
//  double sinRight = std::sin(frontRightWheelAngle);
//  double cosLeft = std::cos(frontLeftWheelAngle);
//  double cosRight = std::cos(frontRightWheelAngle);
//  double alphaLeft = (1 + std::pow(leftInstantaneousCurvature*wheelBase,2))/wheelBase;
//  double alphaRight = (1 + std::pow(rightInstantaneousCurvature*wheelBase,2))/wheelBase;
//  double betaLeft = 2*std::pow(1+leftInstantaneousCurvature*halfTrack,2);
//  double betaRight =2*std::pow(1-rightInstantaneousCurvature*halfTrack,2);
//  double squareAlphaLeft = alphaLeft*alphaLeft;
//  double squareAlphaRight = alphaRight*alphaRight;
//  double squareBetaLeft = betaLeft*betaLeft;
//  double squareBetaRight = betaRight*betaRight;

//  //Todo add correlation between linear speed and curvature
//  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
//  covariance(0,0)+=0.25*wheelSpeedVariance*(cosLeft*cosLeft+cosRight*cosRight);
//  covariance(0,0)+=0.25*wheelAngleVariance*(sinLeft*sinLeft+sinRight*sinRight);
//  covariance(0,2)=-0.5*wheelAngleVariance*(alphaLeft/betaLeft*sinLeft+alphaRight/betaRight*sinRight);
//  covariance(2,0)=covariance(0,2);
//  covariance(2,2)=wheelAngleVariance*(squareAlphaLeft/squareBetaLeft+squareAlphaRight/squareBetaRight);

//  Eigen::MatrixXd Jw = Eigen::MatrixXd::Zero(4,3);
//  Jw(0,0)=1;
//  Jw(1,1)=1;
//  Jw(2,0)=instantaneousCurvature;
//  Jw(2,2)=linearSpeed;
//  Jw(3,2)=1;

//  //Create kinematic frame
//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.speed=linearSpeed;
//  kinematicMeasure.beta=0.;
//  kinematicMeasure.instantaneousCurvature = instantaneousCurvature;
//  kinematicMeasure.angularSpeed=angularSpeed;
//  kinematicMeasure.covariance = Jw*covariance*Jw.transpose();
//  return kinematicMeasure;

//}

////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic2FWS2RWD(const TwoWheelSteeringKinematic & kinematic,
//                                          const OdometryFrame2FWS2RWD & odometryFrame,
//                                          const double &wheelSpeedVariance,
//                                          const double &wheelAngleVariance)
//{

//  const double wheelBase= kinematic.getWheelBase("wheelbase");
//  const double track  = kinematic.getTrack("front_track");
//  const double diffTrack = track - kinematic.getTrack("rear_track");
//  const double & hubCarrierOffset = kinematic.getHubCarrierOffset("front_hub_carrier_offset");

//  //Left information
//  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
//  double leftFrontInstantaneousCurvature = OneAxleSteeringKinematic::computeInstantaneousCurvature(frontLeftWheelAngle,wheelBase);
//  double leftRearInstantaneousCurvature = leftFrontInstantaneousCurvature/(1 + diffTrack*leftFrontInstantaneousCurvature/2.);
//  double rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed/(1-hubCarrierOffset*leftRearInstantaneousCurvature);


//  //Right information
//  const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
//  double rightFrontInstantaneousCurvature = OneAxleSteeringKinematic::computeInstantaneousCurvature(frontRightWheelAngle,wheelBase);
//  double rightRearInstantaneousCurvature = rightFrontInstantaneousCurvature/(1 - diffTrack*rightFrontInstantaneousCurvature/2.);
//  double rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed/(1+hubCarrierOffset*rightRearInstantaneousCurvature);

//  //Compute kinematic values
//  double linearSpeed = 0.5*(rearLeftWheelSpeed+rearRightWheelSpeed);
//  double angularSpeed = 0.5 * (leftFrontInstantaneousCurvature*rearLeftWheelSpeed +rightFrontInstantaneousCurvature*rearRightWheelSpeed);
//  double instantaneousCurvature = TwoWheelSteeringKinematic::computeInstantaneousCurvature(leftFrontInstantaneousCurvature,rightFrontInstantaneousCurvature,track);


//  //Compute covariance matrix
//  double halfTrack = track/2;
//  double alphaLeft = (1 + std::pow(leftFrontInstantaneousCurvature*wheelBase,2));
//  double alphaRight = (1 + std::pow(rightFrontInstantaneousCurvature*wheelBase,2));
//  double betaLeft = 2*std::pow(1+leftFrontInstantaneousCurvature*halfTrack,2);
//  double betaRight =2*std::pow(1-rightFrontInstantaneousCurvature*halfTrack,2);
//  double squareAlphaLeft = alphaLeft*alphaLeft/(wheelBase*wheelBase);
//  double squareAlphaRight = alphaRight*alphaRight/(wheelBase*wheelBase);
//  double squareBetaLeft = betaLeft*betaLeft;
//  double squareBetaRight = betaRight*betaRight;

//  //Todo add correlation between linear speed and curvature
//  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
//  covariance(0,0)=0.5*wheelSpeedVariance;
//  covariance(2,2)=wheelAngleVariance*(squareAlphaLeft/squareBetaLeft+squareAlphaRight/squareBetaRight);

//  Eigen::MatrixXd Jw = Eigen::MatrixXd::Zero(4,3);
//  Jw(0,0)=1;
//  Jw(1,1)=1;
//  Jw(2,0)=instantaneousCurvature;
//  Jw(2,2)=linearSpeed;
//  Jw(3,2)=1;

//  //Create kinematic frame
//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.speed=linearSpeed;
//  kinematicMeasure.beta=0.;
//  kinematicMeasure.instantaneousCurvature = instantaneousCurvature;
//  kinematicMeasure.angularSpeed=angularSpeed;
//  kinematicMeasure.covariance = Jw*covariance*Jw.transpose();
//  return kinematicMeasure;

//}

}
