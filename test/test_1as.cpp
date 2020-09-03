// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_odo/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>

#include <iostream>

//-----------------------------------------------------------------------------
inline void testInverseForward1FAS2FWD(const romea::OneAxleSteeringKinematic::Parameters & parameters,
                                       const romea::OneAxleSteeringConstraints & userConstraints)
{


  for(size_t i=0;i<21;i++)
  {
    double linearSpeed=-1+i*0.1;
    for(size_t j=0;j<21;j++)
    {

      double steeringAngle = -0.5+j*0.05;

      romea::OneAxleSteeringCommand commandFrame;
      commandFrame.longitudinalSpeed=linearSpeed;
      commandFrame.steeringAngle = steeringAngle;

      romea::OneAxleSteeringCommand clampedCommandFrame = romea::clamp(parameters,userConstraints,commandFrame);
      ASSERT_LE(clampedCommandFrame.longitudinalSpeed,userConstraints.getMaximalLinearSpeed());
      ASSERT_GE(clampedCommandFrame.longitudinalSpeed,userConstraints.getMinimalLinearSpeed());
      ASSERT_LE(std::abs(clampedCommandFrame.steeringAngle),userConstraints.getMaximalAbsoluteSteeringAngle());

      romea::OdometryFrame1FAS2FWD odometryFrame;
      romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);

      ASSERT_LE(std::abs(odometryFrame.frontLeftWheelSpeed),parameters.frontMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontRightWheelSpeed),parameters.frontMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontAxleSteeringAngle),parameters.maximalSteeringAngle);

      if(userConstraints.getMaximalLinearSpeed()>=std::numeric_limits<double>::max() &&
         userConstraints.getMinimalLinearSpeed()<=std::numeric_limits<double>::min() &&
         std::abs(commandFrame.longitudinalSpeed-clampedCommandFrame.longitudinalSpeed)>std::numeric_limits<double>::epsilon())
      {

        ASSERT_EQ(romea::near(std::abs(odometryFrame.frontLeftWheelSpeed),parameters.frontMaximalWheelSpeed,0.001)||
                  romea::near(std::abs(odometryFrame.frontRightWheelSpeed),parameters.frontMaximalWheelSpeed,0.001),true);
      }

      romea::OneAxleSteeringMeasure kinematicMeasure;
      romea::inverseKinematic(parameters,odometryFrame,kinematicMeasure);

      ASSERT_EQ(false,std::isnan(linearSpeed));
      ASSERT_EQ(false,std::isnan(steeringAngle));
      ASSERT_NEAR(clampedCommandFrame.longitudinalSpeed,kinematicMeasure.longitudinalSpeed,0.001);
      ASSERT_NEAR(clampedCommandFrame.steeringAngle,kinematicMeasure.steeringAngle,0.001);

    }
  }
}

//-----------------------------------------------------------------------------
inline void testInverseForward1FAS2RWD(const romea::OneAxleSteeringKinematic::Parameters & parameters,
                                       const romea::OneAxleSteeringConstraints & userConstraints)
{

  for(size_t i=0;i<21;i++)
  {
    double linearSpeed=-1+i*0.1;
    for(size_t j=0;j<21;j++)
    {

      double steeringAngle = -0.5+j*0.05;

      romea::OneAxleSteeringCommand commandFrame;
      commandFrame.longitudinalSpeed=linearSpeed;
      commandFrame.steeringAngle = steeringAngle;

      romea::OneAxleSteeringCommand clampedCommandFrame = romea::clamp(parameters,userConstraints,commandFrame);
      ASSERT_LE(clampedCommandFrame.longitudinalSpeed,userConstraints.getMaximalLinearSpeed());
      ASSERT_GE(clampedCommandFrame.longitudinalSpeed,userConstraints.getMinimalLinearSpeed());
      ASSERT_LE(std::abs(clampedCommandFrame.steeringAngle),userConstraints.getMaximalAbsoluteSteeringAngle());

      romea::OdometryFrame1FAS2RWD odometryFrame;
      romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);

      ASSERT_LE(std::abs(odometryFrame.rearLeftWheelSpeed),parameters.rearMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.rearRightWheelSpeed),parameters.rearMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontAxleSteeringAngle),parameters.maximalSteeringAngle);

      romea::OneAxleSteeringMeasure kinematicMeasure;
      romea::inverseKinematic(parameters,odometryFrame,kinematicMeasure);

      ASSERT_NEAR(clampedCommandFrame.longitudinalSpeed,kinematicMeasure.longitudinalSpeed,0.001);
      ASSERT_NEAR(clampedCommandFrame.steeringAngle,kinematicMeasure.steeringAngle,0.001);

    }
  }
}

//  for(size_t i=0;i<21;i++)
//  {
//    double linearSpeed=-1+i*0.1;
//    for(size_t j=0;j<21;j++)
//    {

//      double instantaneousCurvature = -0.5+j*0.05;

//      romea::KinematicCommand kinematicCommand;
//      kinematicCommand.speed =linearSpeed;
//      kinematicCommand.beta =0;
//      kinematicCommand.angularSpeed=instantaneousCurvature*linearSpeed;
//      kinematicCommand.instantaneousCurvature = instantaneousCurvature;

//      romea::OdometryFrame1FAS2FWD odometryFrame = romea::forwardKinematic1FAS2FWD(odometry.getName(),kinematic,kinematicCommand);
//      romea::KinematicMeasure kinematicMeasure = romea::inverseKinematic1FAS2FWD(kinematic,odometry,odometryFrame);

//      ASSERT_EQ(false,std::isnan(linearSpeed));
//      ASSERT_EQ(false,std::isnan(instantaneousCurvature));
//      ASSERT_NEAR(linearSpeed,kinematicMeasure.speed,0.001);
//      ASSERT_NEAR(0,kinematicMeasure.beta,0.001);
//      ASSERT_NEAR(instantaneousCurvature,kinematicMeasure.instantaneousCurvature,0.001);

//      Eigen::Matrix3d covarianceOdometry = Eigen::Matrix3d::Zero();
//      covarianceOdometry(0,0) = odometry.getWheelSpeedVariance();
//      covarianceOdometry(1,1) = odometry.getWheelSpeedVariance();
//      covarianceOdometry(2,2) = odometry.getSteeringAngleVariance();

//      double wheelBase = kinematic.getWheelBase("wheelbase").get();
//      double tanFrontSteeringAngle = instantaneousCurvature*wheelBase;
//      double squareTanFrontSteeringAngle = tanFrontSteeringAngle*tanFrontSteeringAngle;
//      double gamma = (1 + squareTanFrontSteeringAngle)/wheelBase;

//      double frontTrack = kinematic.getTrack("front_track").get();
//      double instantaneousCurvatureHalfTrack_ = instantaneousCurvature*frontTrack/2.;
//      double alphaLeft = 1-instantaneousCurvatureHalfTrack_;
//      double alphaRight =1 + instantaneousCurvatureHalfTrack_;
//      double betaLeft = 1/std::sqrt((alphaLeft*alphaLeft+squareTanFrontSteeringAngle));
//      double betaRight = 1/std::sqrt((alphaRight*alphaRight+squareTanFrontSteeringAngle));
//      double deltaLeft= 0.5*(-(alphaLeft)*frontTrack/2 + 2*tanFrontSteeringAngle*wheelBase)*gamma/(std::pow(betaLeft,3));
//      double deltaRight= 0.5*((alphaRight)*frontTrack/2 + 2*tanFrontSteeringAngle*wheelBase)*gamma/(std::pow(betaRight,3));
//      double delta = deltaLeft + deltaRight;

////      double mu = delta*instantaneousCurvature+linearSpeed*gamma;


////      Eigen::MatrixXd J(4,3);
////      J.row(0) << 0.5*betaLeft , 0.5*betaRight, delta;
////      J.row(1) << 0 , 0 ,0 ;
////      J.row(2) << 0.5*instantaneousCurvature*betaLeft , 0.5*instantaneousCurvature*betaRight, mu;
////      J.row(3) << 0 , 0, gamma;

////      ASSERT_NEAR(0,((J*covarianceOdometry*J.transpose()).array() - kinematicMeasure.covariance.array()).sum(),0.001);


//      Eigen::Matrix3d J(3,3);
//      J.row(0) << 0.5*betaLeft , 0.5*betaRight, delta;
//      J.row(1) << 0 , 0 ,0 ;
//      J.row(2) << 0 , 0, gamma;

//      Eigen::MatrixXd Jw(4,3);
//      Jw(0,0)=1;
//      Jw(1,1)=1;
//      Jw(2,0)=instantaneousCurvature;
//      Jw(2,2)=linearSpeed;
//      Jw(3,2)=1;

//      ASSERT_NEAR(0,((Jw*J*covarianceOdometry*J.transpose()*Jw.transpose()).array() - kinematicMeasure.covariance.array()).sum(),0.001);

//      romea::OneAxleSteeringMeasure oneAxleSteeringMeasure = romea::toOneAxleSteeringMeasure(kinematicMeasure,kinematic);
//      romea::KinematicMeasure kinematicMeasure2 = romea::toKinematicMeasure(oneAxleSteeringMeasure,kinematic);
//      compareKinematicMeasure(kinematicMeasure,kinematicMeasure2);

//    }
//  }
//}


////-----------------------------------------------------------------------------
//inline void testInverseForward1FAS2RWD(const romea::OneAxleSteeringKinematic & kinematic,
//                                       const romea::AxleSteeringOdometry & odometry)
//{


//  for(size_t i=0;i<21;i++)
//  {
//    double linearSpeed=-1+i*0.1;
//    for(size_t j=0;j<21;j++)
//    {

//      double instantaneousCurvature = -0.5+j*0.05;

//      romea::KinematicCommand kinematicCommand;
//      kinematicCommand.speed =linearSpeed;
//      kinematicCommand.beta =0;
//      kinematicCommand.angularSpeed=instantaneousCurvature*linearSpeed;
//      kinematicCommand.instantaneousCurvature = instantaneousCurvature;

//      romea::OdometryFrame1FAS2RWD odometryFrame = romea::forwardKinematic1FAS2RWD(odometry.getName(),kinematic,kinematicCommand);
//      romea::KinematicMeasure kinematicMeasure = romea::inverseKinematic1FAS2RWD(kinematic,odometry,odometryFrame);

//      ASSERT_EQ(false,std::isnan(linearSpeed));
//      ASSERT_EQ(false,std::isnan(instantaneousCurvature));
//      ASSERT_NEAR(linearSpeed,kinematicMeasure.speed,0.001);
//      ASSERT_NEAR(0,kinematicMeasure.beta,0.001);
//      ASSERT_NEAR(instantaneousCurvature,kinematicMeasure.instantaneousCurvature,0.001);

//      Eigen::Matrix3d covarianceOdometry = Eigen::Matrix3d::Zero();
//      covarianceOdometry(0,0) = odometry.getWheelSpeedVariance();
//      covarianceOdometry(1,1) = odometry.getWheelSpeedVariance();
//      covarianceOdometry(2,2) = odometry.getSteeringAngleVariance();

//      double wheelBase = kinematic.getWheelBase("wheelbase").get();
//      double alpha = (1 + std::pow(instantaneousCurvature*wheelBase,2))/wheelBase;

//      Eigen::MatrixXd J(3,3);
//      J.row(0) << 0.5 , 0.5, 0;
//      J.row(1) << 0 , 0 ,0 ;
//      J.row(2) << 0 , 0, alpha;

//      Eigen::MatrixXd Jw(4,3);
//      Jw(0,0)=1;
//      Jw(1,1)=1;
//      Jw(2,0)=instantaneousCurvature;
//      Jw(2,2)=linearSpeed;
//      Jw(3,2)=1;

//      ASSERT_NEAR(0,((Jw*J*covarianceOdometry*J.transpose()*Jw.transpose()).array() - kinematicMeasure.covariance.array()).sum(),0.001);

//      romea::OneAxleSteeringMeasure oneAxleSteeringMeasure = romea::toOneAxleSteeringMeasure(kinematicMeasure,kinematic);
//      romea::KinematicMeasure kinematicMeasure2 = romea::toKinematicMeasure(oneAxleSteeringMeasure,kinematic);
//      compareKinematicMeasure(kinematicMeasure,kinematicMeasure2);
//    }
//  }
//}


//TEST(Test1AS, testInverseForward1FAS2RWD)
//{

//  {
//    romea::OneAxleSteeringKinematic kinematic(1.4,1.2,1.2);
//    romea::AxleSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForward1FAS2RWD(kinematic,odometry);
//  }

//  {
//    romea::OneAxleSteeringKinematic kinematic(2.5,1.4,1.4);
//    romea::AxleSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForward1FAS2RWD(kinematic,odometry);
//  }

//}

TEST(testInverseForward1FAS2FWD,SameTrack)
{


  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontTrack=1.2;
  parameters.rearTrack=1.2;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;

  testInverseForward1FAS2FWD(parameters,
                             userConstraints);
}

TEST(testInverseForward1FAS2FWD,DiffTrack)
{
  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontTrack=1.2;
  parameters.rearTrack=1.6;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2FWD(parameters,
                             userConstraints);
}




TEST(testInverseForward1FAS2FWD,HubOffset)
{

  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontTrack=1.2;
  parameters.rearTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2FWD(parameters,
                             userConstraints);
}



TEST(testInverseForward1FAS2FWD, MecanicalLimits)
{

  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.frontMaximalWheelSpeed=1;
  parameters.maximalSteeringAngle=0.3;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2FWD(parameters,
                             userConstraints);
}

TEST(testInverseForward1FAS2FWD, UserLimits)
{
  romea::OneAxleSteeringConstraints userConstraints(-0.4,0.8,0.25);

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.frontMaximalWheelSpeed=1;
  parameters.maximalSteeringAngle=0.3;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;


  testInverseForward1FAS2FWD(parameters,
                             userConstraints);
}



TEST(testInverseForward1FAS2RWD,SameTrack)
{
  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontTrack=1.2;
  parameters.rearTrack=1.2;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;


  testInverseForward1FAS2RWD(parameters,
                             userConstraints);
}

TEST(testInverseForward1FAS2RWD,DiffTrack)
{
  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.8;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;


  testInverseForward1FAS2RWD(parameters,
                             userConstraints);
}




TEST(testInverseForward1FAS2RWD,HubOffset)
{

  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.8;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2RWD(parameters,
                             userConstraints);
}



TEST(testInverseForward1FAS2RWD, MecanicalLimits)
{
  romea::OneAxleSteeringConstraints userConstraints;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.8;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.frontMaximalWheelSpeed=1;
  parameters.maximalSteeringAngle=0.3;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2RWD(parameters,
                             userConstraints);
}

TEST(testInverseForward1FAS2RWD, UserLimits)
{

  romea::OneAxleSteeringConstraints userConstraints(-0.4,0.9,0.25);

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.8;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.frontMaximalWheelSpeed=1;
  parameters.maximalSteeringAngle=0.3;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;


  testInverseForward1FAS2RWD(parameters,
                             userConstraints);
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
