// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_odo/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_odo/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

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


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
