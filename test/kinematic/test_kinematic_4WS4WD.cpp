// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"
#include "romea_core_common/math/Matrix.hpp"



//-----------------------------------------------------------------------------
inline void testInverseForward4WS4WD(const romea::FourWheelSteeringKinematic::Parameters & parameters,
                                     const romea::TwoAxleSteeringCommandLimits & userLimits)
{
  for(size_t i=0;i<21;i++)
  {
    double linearSpeed=-1+i*0.1;
    for(size_t j=0;j<21;j++)
    {
      double frontSteeringAngle = -0.5+j*0.05;

      for(size_t k=0;k<21;k++)
      {
        double rearSteeringAngle = -0.5+k*0.05;


        romea::TwoAxleSteeringCommand commandFrame;
        commandFrame.longitudinalSpeed=linearSpeed;
        commandFrame.frontSteeringAngle = frontSteeringAngle;
        commandFrame.rearSteeringAngle = rearSteeringAngle;

        romea::TwoAxleSteeringCommand clampedCommandFrame = romea::clamp(parameters,userLimits,commandFrame);
        ASSERT_LE(clampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.upper());
        ASSERT_GE(clampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.lower());
        ASSERT_LE(std::abs(clampedCommandFrame.frontSteeringAngle),userLimits.frontSteeringAngle.upper());
        ASSERT_LE(std::abs(clampedCommandFrame.rearSteeringAngle),userLimits.frontSteeringAngle.upper());


        romea::OdometryFrame4WS4WD odometryFrame;
        romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);

        ASSERT_LE(std::abs(odometryFrame.frontLeftWheelLinearSpeed),parameters.maximalWheelLinearSpeed+0.01);
        ASSERT_LE(std::abs(odometryFrame.frontRightWheelLinearSpeed),parameters.maximalWheelLinearSpeed+0.01);
        ASSERT_LE(std::abs(odometryFrame.rearLeftWheelLinearSpeed),parameters.maximalWheelLinearSpeed+0.01);
        ASSERT_LE(std::abs(odometryFrame.frontRightWheelLinearSpeed),parameters.maximalWheelLinearSpeed+0.01);
        ASSERT_LE(std::abs(odometryFrame.frontLeftWheelSteeringAngle),parameters.maximalWheelSteeringAngle+0.01);
        ASSERT_LE(std::abs(odometryFrame.frontRightWheelSteeringAngle),parameters.maximalWheelSteeringAngle+0.01);
        ASSERT_LE(std::abs(odometryFrame.rearLeftWheelSteeringAngle),parameters.maximalWheelSteeringAngle+0.01);
        ASSERT_LE(std::abs(odometryFrame.frontRightWheelSteeringAngle),parameters.maximalWheelSteeringAngle+0.01);


        romea::TwoAxleSteeringMeasure kinematicMeasure;
        romea::inverseKinematic(parameters,odometryFrame,kinematicMeasure);

        ASSERT_NEAR(clampedCommandFrame.longitudinalSpeed,kinematicMeasure.longitudinalSpeed,0.001);
        ASSERT_NEAR(clampedCommandFrame.frontSteeringAngle,kinematicMeasure.frontSteeringAngle,0.001);
        ASSERT_NEAR(clampedCommandFrame.rearSteeringAngle,kinematicMeasure.rearSteeringAngle,0.001);
      }
    }
  }
}

//-----------------------------------------------------------------------------
inline double radius(double x, double y, double theta, double xw, double yw, double thetaw)
{

  double vx1= cos(theta+M_PI/2);
  double vy1= sin(theta+M_PI/2);

  double vx2 = cos(thetaw+M_PI/2);
  double vy2 = sin(thetaw+M_PI/2);
  double vx21=(xw-x);
  double vy21=(yw-y);

  return (vx21*vy2 - vy21*vx2)/(vx1*vy2-vy1*vx2);
}

//-----------------------------------------------------------------------------
inline void testCircularMovement(romea::FourWheelSteeringKinematic::Parameters &parameters,
                                 const double & v,
                                 const double & R)
{
  double dt = 0.0001;
  double K = 1/R;

  size_t n = 2*M_PI*std::abs(R) / (v*dt);

  const double frontWheelTrack=parameters.wheelTrack;
  const double rearWheelTrack=parameters.wheelTrack;
  const double frontWheelBase=parameters.frontWheelBase;
  const double rearWheelBase= parameters.rearWheelBase;

  romea::TwoAxleSteeringCommand command;
  command.longitudinalSpeed=v;
  command.frontSteeringAngle = std::atan(K*frontWheelBase);
  command.rearSteeringAngle = -std::atan(K*rearWheelBase);

//  std::cout <<  command.frontSteeringAngle << " "<< K <<" "<< frontWheelBase << " "<< K*frontWheelBase<<std::endl;
//  std::cout <<  command.frontSteeringAngle << " "<< K <<" "<< rearWheelBase << " "<< K*rearWheelBase<<std::endl;

  romea::OdometryFrame4WS4WD odometryFrame;
  romea::forwardKinematic(parameters,command,odometryFrame);

  double xfl =  frontWheelBase;
  double yfl =  frontWheelTrack/2.;
  double xfr =  frontWheelBase;
  double yfr = -frontWheelTrack/2.;
  double xrl = -rearWheelBase;
  double yrl =  rearWheelTrack/2.;
  double xrr = -rearWheelBase;
  double yrr = -rearWheelTrack/2.;


  for(size_t i =1;i<n; i++){

    //Vehicle orientation and center
    double theta=std::atan2(0.5*(yfl+yfr) - 0.5*(yrl+yrr),0.5*(xfl+xfr) - 0.5*(xrl+xrr));
    double x = 0.25*(xfl+xfr+xrl+xrr);
    double y = 0.25*(yfl+yfr+yrl+yrr);

    //Wheel orientatons
    double thetafl = theta + odometryFrame.frontLeftWheelSteeringAngle;
    double thetafr = theta + odometryFrame.frontRightWheelSteeringAngle;
    double thetarl = theta + odometryFrame.rearLeftWheelSteeringAngle;
    double thetarr = theta + odometryFrame.rearRightWheelSteeringAngle;

    //Wheel speeds
    double vfl =odometryFrame.frontLeftWheelLinearSpeed;
    double vfr =odometryFrame.frontRightWheelLinearSpeed;
    double vrl =odometryFrame.rearLeftWheelLinearSpeed;
    double vrr =odometryFrame.rearRightWheelLinearSpeed;

    //New wheel positions
    xfl += std::cos(thetafl)*vfl*dt;
    yfl += std::sin(thetafl)*vfl*dt;
    xfr += std::cos(thetafr)*vfr*dt;
    yfr += std::sin(thetafr)*vfr*dt;
    xrl += std::cos(thetarl)*vrl*dt;
    yrl += std::sin(thetarl)*vrl*dt;
    xrr += std::cos(thetarr)*vrr*dt;
    yrr += std::sin(thetarr)*vrr*dt;

    //Compute radius of curvature
    double rfl = radius(x,y,theta,xfl,yfl,thetafl);
    double rfr = radius(x,y,theta,xfr,yfr,thetafr);
    double rrl = radius(x,y,theta,xrl,yrl,thetarl);
    double rrr = radius(x,y,theta,xrr,yrr,thetarr);

    ASSERT_NEAR(rfl,R,0.1);
    ASSERT_NEAR(rfr,R,0.1);
    ASSERT_NEAR(rrl,R,0.1);
    ASSERT_NEAR(rrr,R,0.1);

  }
  ASSERT_NEAR(xfl, frontWheelBase ,0.01);
  ASSERT_NEAR(yfl, frontWheelTrack/2.,0.01);
  ASSERT_NEAR(xfr, frontWheelBase ,0.01);
  ASSERT_NEAR(yfr,-frontWheelTrack/2.,0.01);
  ASSERT_NEAR(xrl,-rearWheelBase,0.01);
  ASSERT_NEAR(yrl, rearWheelTrack/2.,0.01);
  ASSERT_NEAR(xrr,-rearWheelBase ,0.01);
  ASSERT_NEAR(yrr,-rearWheelTrack/2.,0.01);
}


TEST(testInverseForward4WS4WD,SameWheelbase)
{

  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::FourWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.wheelTrack = 1.2;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.wheelSteeringAngleVariance=0.02*0.02;



  testInverseForward4WS4WD(parameters,
                           userLimits);
}


TEST(testInverseForward4WS4WD,DiffWheelbase)
{

  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::FourWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1;
  parameters.rearWheelBase = 0.7;
  parameters.wheelTrack = 1.2;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.wheelSteeringAngleVariance=0.02*0.02;



  testInverseForward4WS4WD(parameters,
                           userLimits);
}


TEST(testInverseForward4WS4WD,HubOffset)
{
  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::FourWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1;
  parameters.rearWheelBase = 0.7;
  parameters.wheelTrack = 1.2;
  parameters.hubCarrierOffset=0.1;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.wheelSteeringAngleVariance=0.02*0.02;

  testInverseForward4WS4WD(parameters,
                           userLimits);
}


//TEST(testInverseForward4WS4WD,MechanicalLimits)
//{
//  romea::TwoAxleSteeringCommandLimits userLimits;

//  romea::FourWheelSteeringKinematic::Parameters parameters;
//  parameters.frontWheelBase = 1;
//  parameters.rearWheelBase = 0.7;
//  parameters.wheelTrack = 1.2;
//  parameters.hubCarrierOffset=0.1;
//  parameters.maximalWheelAngle =0.3;
//  parameters.maximalWheelSpeed =1.;

//  testInverseForward4WS4WD(parameters,
//                           userLimits);
//}


////-----------------------------------------------------------------------------
//TEST(Test4WS, testInverseForward)
//{

//  {
//    romea::FourWheelSteeringKinematic kinematic(1.4,1.4,1.1,1.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForward(kinematic,odometry);
//  }


//  {
//    romea::FourWheelSteeringKinematic kinematic(1.4,1.2,1.1,1.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForward(kinematic,odometry);
//  }

//}

////-----------------------------------------------------------------------------
//TEST(Test4WS, testInverseForwardTranslation)
//{

//  {
//    romea::FourWheelSteeringKinematic kinematic1(1.4,1.4,1.1,1.1);
//    romea::FourWheelSteeringKinematic kinematic2(2.8,0,1.1,1.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForwardTranslation(kinematic1,kinematic2,odometry);
//  }


//  {
//    romea::FourWheelSteeringKinematic kinematic1(1.4,1.2,1.1,1.1);
//    romea::FourWheelSteeringKinematic kinematic2(2.6,0,1.1,1.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForwardTranslation(kinematic1,kinematic2,odometry);
//  }

//}

////-----------------------------------------------------------------------------
//TEST(Test4WS, testInverseForwardHub)
//{

//  {
//    romea::FourWheelSteeringKinematic kinematic(1.4,1.4,1.1,1.1,0.1,0.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForward(kinematic,odometry);
//  }


//  {
//    romea::FourWheelSteeringKinematic kinematic(1.4,1.2,1.1,1.1,0.1,0.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForward(kinematic,odometry);
//  }

//}


////-----------------------------------------------------------------------------
//TEST(Test4WS, testInverseForwardFake2WS)
//{

//  romea::TwoWheelSteeringKinematic kinematic2WS(2.8,1.1,1.1,0.1,0.1);
//  romea::FourWheelSteeringKinematic kinematic4WS(2.8,0,1.1,1.1,0.1,0.1);
//  romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);

//  for(size_t i=0;i<21;i++)
//  {
//    double linearSpeed=-1+i*0.1;
//    for(size_t j=0;j<21;j++)
//    {

//      double instantaneousCurvature = -0.5+j*0.05;
//      double angularSpeed =instantaneousCurvature * linearSpeed;
//      double extendedTrack = kinematic4WS.getTrack("rear_track").get()+kinematic4WS.getHubCarrierOffset("rear_hub_carrier_offset").get();

//      romea::KinematicCommand kinematicCommand;
//      kinematicCommand.speed =linearSpeed;
//      kinematicCommand.beta =0;
//      kinematicCommand.angularSpeed=instantaneousCurvature*linearSpeed;
//      kinematicCommand.instantaneousCurvature = instantaneousCurvature;

//      romea::OdometryFrame2FWS2FWD odometryFrame2WS = romea::forwardKinematic2FWS2FWD(odometry.getName(),kinematic2WS,kinematicCommand);


//      romea::OdometryFrame4WS4WD odometryFrame4WS;
//      odometryFrame4WS.frontLeftWheelSpeed=odometryFrame2WS.frontLeftWheelSpeed;
//      odometryFrame4WS. frontLeftWheelAngle= odometryFrame2WS.frontLeftWheelAngle;
//      odometryFrame4WS.frontRightWheelSpeed=odometryFrame2WS.frontRightWheelSpeed;
//      odometryFrame4WS.frontRightWheelAngle=odometryFrame2WS.frontRightWheelAngle;
//      odometryFrame4WS.rearLeftWheelSpeed= romea::SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
//                                                                                               angularSpeed,
//                                                                                               extendedTrack);
//      odometryFrame4WS.rearLeftWheelAngle=0;
//      odometryFrame4WS.rearRightWheelSpeed=romea::SkidSteeringKinematic::computeRightWheelSpeed(linearSpeed,
//                                                                                                angularSpeed,
//                                                                                                extendedTrack);
//      odometryFrame4WS.rearRightWheelAngle=0;
//      odometryFrame4WS.emitterName =odometryFrame2WS.emitterName;


//      romea::KinematicMeasure kinematicMeasure = romea::inverseKinematic4WS4WD(kinematic4WS,odometry,odometryFrame4WS);

//      ASSERT_EQ(false,std::isnan(linearSpeed));
//      ASSERT_EQ(false,std::isnan(instantaneousCurvature));
//      ASSERT_NEAR(linearSpeed,kinematicMeasure.speed,0.001);
//      ASSERT_NEAR(0,kinematicMeasure.beta,0.001);
//      ASSERT_NEAR(instantaneousCurvature,kinematicMeasure.instantaneousCurvature,0.001);

//    }
//  }
//}

//-----------------------------------------------------------------------------
TEST(Test4WS, testCircularMovement)
{

  {
    romea::FourWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase =1.4;
    parameters.rearWheelBase = 1.4;
    parameters.wheelTrack=1.1;
    testCircularMovement(parameters,1,3);
  }


  {
    romea::FourWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase =1.4;
    parameters.rearWheelBase = 1.4;
    parameters.wheelTrack=1.2;
    testCircularMovement(parameters,1,-3);
  }

}

////-----------------------------------------------------------------------------
//TEST(Test4WS, testInverseForwardTranslationHub)
//{

//  {
//    romea::FourWheelSteeringKinematic kinematic1(1.4,1.4,1.1,1.1,0.1,0.1);
//    romea::FourWheelSteeringKinematic kinematic2(2.8,0,1.1,1.1,0.1,0.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForwardTranslation(kinematic1,kinematic2,odometry);
//  }


//  {
//    romea::FourWheelSteeringKinematic kinematic1(1.4,1.2,1.1,1.1,0.1,0.1);
//    romea::FourWheelSteeringKinematic kinematic2(2.6,0,1.1,1.1,0.1,0.1);
//    romea::WheelSteeringOdometry odometry(10,0.1,0.01,0.02,0.01);
//    testInverseForwardTranslation(kinematic1,kinematic2,odometry);
//  }

//}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
