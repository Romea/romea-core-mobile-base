// gtest
#include <gtest/gtest.h>
#include <limits>

//romea
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>


//-----------------------------------------------------------------------------
inline void testInverseForward2WD(const romea::SkidSteeringKinematic::Parameters  & parameters,
                                  const romea::SkidSteeringCommandLimits & userLimits)
{

  for(size_t i=0;i<21;i++)
  {
    double linearSpeed=-1+i*0.1;
    for(size_t j=0;j<21;j++)
    {

      double angularSpeed = -0.5+j*0.05;

      romea::SkidSteeringCommand commandFrame;
      commandFrame.longitudinalSpeed = linearSpeed;
      commandFrame.angularSpeed=angularSpeed;

      romea::SkidSteeringCommand clampedCommandFrame = romea::clamp(parameters,userLimits,commandFrame);

      ASSERT_LE(clampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.upper());
      ASSERT_GE(clampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.lower());
      ASSERT_LE(std::abs(clampedCommandFrame.angularSpeed),userLimits.angularSpeed.upper());

      romea::OdometryFrame2WD odometryFrame;
      romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);

      ASSERT_LE(std::abs(odometryFrame.leftWheelSpeed),parameters.maximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.rightWheelSpeed),parameters.maximalWheelSpeed);

      if(userLimits.longitudinalSpeed.upper()>=std::numeric_limits<double>::max() &&
         userLimits.longitudinalSpeed.lower()<=std::numeric_limits<double>::min() &&
         std::abs(commandFrame.longitudinalSpeed-clampedCommandFrame.longitudinalSpeed)>std::numeric_limits<double>::epsilon())
      {

        ASSERT_EQ(romea::near(std::abs(odometryFrame.leftWheelSpeed),parameters.maximalWheelSpeed,0.001)||
                  romea::near(std::abs(odometryFrame.rightWheelSpeed),parameters.maximalWheelSpeed,0.001),true);
      }


      romea::SkidSteeringMeasure kinematicMeasure;
      romea::inverseKinematic(parameters,odometryFrame,kinematicMeasure);

      ASSERT_NEAR(clampedCommandFrame.longitudinalSpeed,kinematicMeasure.longitudinalSpeed,0.001);
      ASSERT_NEAR(clampedCommandFrame.angularSpeed,kinematicMeasure.angularSpeed,0.001);

    }
  }
}

//-----------------------------------------------------------------------------
inline void testCinematicClamp(const romea::SkidSteeringKinematic::Parameters  & parameters,
                               const romea::SkidSteeringCommandLimits & userLimits)
{

  double dt =0.1;
  for(size_t i=0;i<21;i++)
  {
    double firstLinearSpeed=-1+i*0.1;
    for(size_t j=0;j<21;j++)
    {
      double firstAngularSpeed = -0.5+j*0.05;
      for(size_t k=0;k<21;k++)
      {
        double secondLinearSpeed=-1+k*0.1;
        for(size_t l=0;l<21;l++)
        {
          double secondAngularSpeed = -0.5+l*0.05;

          romea::SkidSteeringCommand firstCommandFrame;
          firstCommandFrame.longitudinalSpeed = firstLinearSpeed;
          firstCommandFrame.angularSpeed= firstAngularSpeed;

          romea::SkidSteeringCommand firstClampedCommandFrame = romea::clamp(parameters,userLimits,firstCommandFrame);

          romea::SkidSteeringCommand secondCommandFrame;
          secondCommandFrame.longitudinalSpeed = secondLinearSpeed;
          secondCommandFrame.angularSpeed= secondAngularSpeed;

          romea::SkidSteeringCommand secondClampedCommandFrame = romea::clamp(parameters,userLimits,secondCommandFrame);
          secondClampedCommandFrame = romea::clamp(parameters,firstClampedCommandFrame,secondClampedCommandFrame,dt);

          ASSERT_LE(secondClampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.upper());
          ASSERT_GE(secondClampedCommandFrame.longitudinalSpeed,userLimits.longitudinalSpeed.lower());
          ASSERT_LE(std::abs(secondClampedCommandFrame.angularSpeed),userLimits.angularSpeed.upper());

          romea::OdometryFrame2WD firstOdometryFrame;
          romea::forwardKinematic(parameters,firstClampedCommandFrame,firstOdometryFrame);

          romea::OdometryFrame2WD secondOdometryFrame;
          romea::forwardKinematic(parameters,secondClampedCommandFrame,secondOdometryFrame);

          ASSERT_LE(std::abs((secondOdometryFrame.leftWheelSpeed-firstOdometryFrame.leftWheelSpeed)),parameters.maximalWheelAcceleration*dt+0.000001);
          ASSERT_LE(std::abs((secondOdometryFrame.rightWheelSpeed-firstOdometryFrame.rightWheelSpeed)),parameters.maximalWheelAcceleration*dt+0.000001);

        }
      }
    }
  }
}


TEST(testInverseForward2D, NoLimit)
{
  romea::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 0.6;
  parameters.wheelSpeedVariance = 0.1*0.1;
  testInverseForward2WD(parameters,romea::SkidSteeringCommandLimits());
}

TEST(testInverseForward2D, MechanicalLimits)
{
  romea::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 2.5;
  parameters.maximalWheelSpeed = 1;
  parameters.wheelSpeedVariance = 0.1*0.1;
  testInverseForward2WD(parameters,romea::SkidSteeringCommandLimits());
}

TEST(testInverseForward2D, UserLimits)
{
  romea::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 2.5;
  parameters.maximalWheelSpeed=1;
  parameters.wheelSpeedVariance = 0.1*0.1;
  testInverseForward2WD(parameters,romea::SkidSteeringCommandLimits(-0.4,0.8,0.5));
}

TEST(testCinematicClamp,MechanicalLimits)
{
    romea::SkidSteeringKinematic::Parameters parameters;
    parameters.wheelTrack = 2.5;
    parameters.maximalWheelSpeed = 1;
    parameters.maximalWheelAcceleration = 1;
    testCinematicClamp(parameters,romea::SkidSteeringCommandLimits());
}

TEST(testCinematicClamp,UserLimits)
{
    romea::SkidSteeringKinematic::Parameters parameters;
    parameters.wheelTrack = 2.5;
    parameters.maximalWheelSpeed = 1;
    parameters.maximalWheelAcceleration = 1;
    testCinematicClamp(parameters,romea::SkidSteeringCommandLimits(-0.4,0.8,0.5));
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
