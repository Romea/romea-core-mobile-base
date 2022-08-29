
// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

#include <iostream>

//-----------------------------------------------------------------------------
inline void testInverseForward1FAS2FWD(const romea::OneAxleSteeringKinematic::Parameters & parameters,
                                       const romea::OneAxleSteeringCommandLimits & userLimits)
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

      romea::OneAxleSteeringCommand clampedCommandFrame = romea::clamp(parameters,userLimits,commandFrame);
      ASSERT_LE(clampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.upper());
      ASSERT_GE(clampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.lower());
      ASSERT_LE(std::abs(clampedCommandFrame.steeringAngle),userLimits.steeringAngle.upper());

      romea::OdometryFrame1FAS2FWD odometryFrame;
      romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);

      ASSERT_LE(std::abs(odometryFrame.frontLeftWheelLinearSpeed),parameters.frontMaximalWheelLinearSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontRightWheelLinearSpeed),parameters.frontMaximalWheelLinearSpeed);
      ASSERT_LE(std::abs(odometryFrame.frontAxleSteeringAngle),parameters.maximalSteeringAngle);

      if(userLimits.longitudinalSpeed.upper()>=std::numeric_limits<double>::max() &&
         userLimits.longitudinalSpeed.lower()<=std::numeric_limits<double>::min() &&
         std::abs(commandFrame.longitudinalSpeed-clampedCommandFrame.longitudinalSpeed)>std::numeric_limits<double>::epsilon())
      {

        ASSERT_EQ(romea::near(std::abs(odometryFrame.frontLeftWheelLinearSpeed),parameters.frontMaximalWheelLinearSpeed,0.001)||
                  romea::near(std::abs(odometryFrame.frontRightWheelLinearSpeed),parameters.frontMaximalWheelLinearSpeed,0.001),true);
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


  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontWheelTrack=1.2;
  parameters.rearWheelTrack=1.2;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;

  testInverseForward1FAS2FWD(parameters,
                             userLimits);
}

TEST(testInverseForward1FAS2FWD,DiffTrack)
{
  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontWheelTrack=1.2;
  parameters.rearWheelTrack=1.6;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2FWD(parameters,
                             userLimits);
}




TEST(testInverseForward1FAS2FWD,HubOffset)
{

  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 0.7;
  parameters.rearWheelBase= 0.7;
  parameters.frontWheelTrack=1.2;
  parameters.rearWheelTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2FWD(parameters,
                             userLimits);
}



TEST(testInverseForward1FAS2FWD, MecanicalLimits)
{

  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontWheelTrack=1.4;
  parameters.rearWheelTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.frontMaximalWheelLinearSpeed=1;
  parameters.maximalSteeringAngle=0.3;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;



  testInverseForward1FAS2FWD(parameters,
                             userLimits);
}

TEST(testInverseForward1FAS2FWD, UserLimits)
{
  romea::OneAxleSteeringCommandLimits userLimits(-0.4,0.8,0.25);

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase= 1.25;
  parameters.rearWheelBase= 1.25;
  parameters.frontWheelTrack=1.4;
  parameters.rearWheelTrack=1.6;
  parameters.frontHubCarrierOffset=0.1;
  parameters.rearHubCarrierOffset=0.1;
  parameters.frontMaximalWheelLinearSpeed=1;
  parameters.maximalSteeringAngle=0.3;
  parameters.wheelLinearSpeedVariance=0.1*0.1;
  parameters.steeringAngleVariance =0.02*0.02;


  testInverseForward1FAS2FWD(parameters,
                             userLimits);
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
