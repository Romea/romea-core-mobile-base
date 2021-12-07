// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_odo/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_core_odo/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp"
#include "romea_core_odo/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_odo/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include <romea_core_common/math/Algorithm.hpp>


//-----------------------------------------------------------------------------
inline void testInverseForward2FWS4WD(const romea::TwoWheelSteeringKinematic::Parameters & parameters,
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

      romea::OdometryFrame2FWS4WD odometryFrame;
      romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);


      ASSERT_LE(std::abs(odometryFrame.frontLeftWheelSpeed),parameters.frontMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontRightWheelSpeed),parameters.frontMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.rearLeftWheelSpeed),parameters.rearMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.rearRightWheelSpeed),parameters.rearMaximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontLeftWheelAngle),parameters.maximalWheelAngle);
      ASSERT_LE(std::abs(odometryFrame.frontRightWheelAngle),parameters.maximalWheelAngle);

      if(userConstraints.getMaximalLinearSpeed()>=std::numeric_limits<double>::max() &&
         userConstraints.getMinimalLinearSpeed()<=std::numeric_limits<double>::min() &&
         std::abs(commandFrame.longitudinalSpeed-clampedCommandFrame.longitudinalSpeed)>std::numeric_limits<double>::epsilon())
      {
//        std::cout << odometryFrame.rearLeftWheelSpeed <<" "<< parameters.rearMaximalWheelSpeed<< std::endl;
//        std::cout << odometryFrame.rearRightWheelSpeed <<" "<< parameters.rearMaximalWheelSpeed<< std::endl;
//        ASSERT_EQ(romea::near(std::abs(odometryFrame.rearLeftWheelSpeed),parameters.rearMaximalWheelSpeed,0.001)||
//                  romea::near(std::abs(odometryFrame.rearRightWheelSpeed),parameters.rearMaximalWheelSpeed,0.001),true);

        ASSERT_EQ(romea::near(std::abs(odometryFrame.frontLeftWheelSpeed),parameters.frontMaximalWheelSpeed,0.001)||
                  romea::near(std::abs(odometryFrame.frontRightWheelSpeed),parameters.frontMaximalWheelSpeed,0.001),true);

      }


      romea::OneAxleSteeringMeasure kinematicMeasure;
      romea::inverseKinematic(parameters,odometryFrame,kinematicMeasure);

      ASSERT_NEAR(clampedCommandFrame.longitudinalSpeed,kinematicMeasure.longitudinalSpeed,0.001);
      ASSERT_NEAR(clampedCommandFrame.steeringAngle,kinematicMeasure.steeringAngle,0.001);

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
inline void testCircularMovement(romea::TwoWheelSteeringKinematic::Parameters & parameters,
                                 const double & v,
                                 const double & R)
{

  const double wheelBase= parameters.frontWheelBase+parameters.rearWheelBase;
  const double frontTrack  = parameters.frontTrack;
  const double rearTrack = parameters.rearTrack;

  double dt = 0.0001;
  double K = 1/R;

  size_t n = 2*M_PI*std::abs(R) / (v*dt);

  romea::OneAxleSteeringCommand command;
  command.longitudinalSpeed=v;
  command.steeringAngle=std::atan(K*wheelBase);


  double deltafl,deltafr;
  double vrl,vrr,vfl,vfr;

  romea::OdometryFrame2FWS4WD odometryFrame;
  romea::forwardKinematic(parameters,command,odometryFrame);

  deltafl = odometryFrame.frontLeftWheelAngle;
  deltafr = odometryFrame.frontRightWheelAngle;

  vfl =odometryFrame.frontLeftWheelSpeed;
  vfr =odometryFrame.frontRightWheelSpeed;

  vrl =odometryFrame.rearLeftWheelSpeed;
  vrr =odometryFrame.rearRightWheelSpeed;

  double xfl =  wheelBase;
  double yfl =  frontTrack/2.;
  double xfr =  wheelBase;
  double yfr = -frontTrack/2.;
  double xrl =  0;
  double yrl =  rearTrack/2.;
  double xrr =  0;
  double yrr = -rearTrack/2.;


  for(size_t i =1;i<n; i++){

    //Vehicle orientation and center
    double theta=std::atan2(0.5*(yfl+yfr) - 0.5*(yrl+yrr),0.5*(xfl+xfr) - 0.5*(xrl+xrr));
    double x = 0.5*(xrl+xrr);
    double y = 0.5*(yrl+yrr);

    //Wheel orientatons
    double thetarl = theta ;
    double thetarr = theta ;
    double thetafl = theta + deltafl;
    double thetafr = theta + deltafr;

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
    ASSERT_NEAR(rfl,R,0.001);
    ASSERT_NEAR(rfr,R,0.001);

  }
  ASSERT_NEAR(xfl, wheelBase ,0.01);
  ASSERT_NEAR(yfl, frontTrack/2.,0.01);
  ASSERT_NEAR(xfr, wheelBase ,0.01);
  ASSERT_NEAR(yfr,-frontTrack/2.,0.01);
  ASSERT_NEAR(xrl,0,0.01);
  ASSERT_NEAR(yrl, rearTrack/2.,0.01);
  ASSERT_NEAR(xrr,0,0.01);
  ASSERT_NEAR(yrr,-rearTrack/2.,0.01);
}




TEST(testInverseForward2FWS4WD, SameTrack)
{

  romea::OneAxleSteeringConstraints userConstraints;

  romea::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.8;
  parameters.rearWheelBase= 0.8;
  parameters.frontTrack=1.1;
  parameters.rearTrack=1.1;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.wheelAngleVariance=0.02*0.02;


  testInverseForward2FWS4WD(parameters,
                             userConstraints);
}

TEST(testInverseForward2FWS4WD,DiffTrack)
{
  romea::OneAxleSteeringConstraints userConstraints;

  romea::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.8;
  parameters.rearWheelBase= 0.8;
  parameters.frontTrack=1.5;
  parameters.rearTrack=1.2;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.wheelAngleVariance=0.02*0.02;

  testInverseForward2FWS4WD(parameters,
                             userConstraints);
}

TEST(testInverseForward2FWS4WD,HubOffset)
{
  const double wheelSpeedVariance=0.1*0.1;
  const double steeringAngleVariance =0.02*0.02;

  romea::OneAxleSteeringConstraints userConstraints;

  romea::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontTrack=1.1;
  parameters.rearTrack=1.4;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.wheelAngleVariance=0.02*0.02;

  testInverseForward2FWS4WD(parameters,
                             userConstraints);
}

TEST(testInverseForward2FWS4WD, DISABLED_MecanicalLimits)
{

  romea::OneAxleSteeringConstraints userConstraints;

  romea::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.rearMaximalWheelSpeed=1;
  parameters.maximalWheelAngle=0.3;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.wheelAngleVariance=0.02*0.02;


  testInverseForward2FWS4WD(parameters,
                             userConstraints);
}

TEST(testInverseForward2FWS4WD, UserLimits)
{
  romea::OneAxleSteeringConstraints userConstraints(-0.4,0.8,0.25);

  romea::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontTrack=1.4;
  parameters.rearTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.rearMaximalWheelSpeed=1;
  parameters.maximalWheelAngle=0.3;
  parameters.wheelSpeedVariance=0.1*0.1;
  parameters.wheelAngleVariance=0.02*0.02;


  testInverseForward2FWS4WD(parameters,
                             userConstraints);
}


//-----------------------------------------------------------------------------
TEST(Test2WS, testCircularMovement2FWS4WD)
{

  {
    romea::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase=1.4;
    parameters.rearWheelBase=0;
    parameters.frontTrack=1.2;
    parameters.rearTrack=1.2;
    testCircularMovement(parameters,1,3);
  }

  {
    romea::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase=1.4;
    parameters.rearWheelBase=0;
    parameters.frontTrack=0.8;
    parameters.rearTrack=0.8;
    testCircularMovement(parameters,1,-3);
  }

  {
    romea::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase=1.4;
    parameters.rearWheelBase=0;
    parameters.frontTrack=0.8;
    parameters.rearTrack=1.2;
    testCircularMovement(parameters,1,3);
  }

  {
    romea::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase=1.4;
    parameters.rearWheelBase=0;
    parameters.frontTrack=0.8;
    parameters.rearTrack=1.2;
    testCircularMovement(parameters,1,-3);
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

