// gtest
#include <gtest/gtest.h>

//romea
#include "romea_odo/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include "romea_odo/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>


//-----------------------------------------------------------------------------
inline void testInverseForward2WD(const romea::SkidSteeringKinematic::Parameters  & parameters,
                                  const romea::SkidSteeringConstraints & userConstraints)
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

      romea::SkidSteeringCommand clampedCommandFrame = romea::clamp(parameters,userConstraints,commandFrame);

      ASSERT_LE(clampedCommandFrame.longitudinalSpeed,userConstraints.getMaximalLinearSpeed());
      ASSERT_GE(clampedCommandFrame.longitudinalSpeed,userConstraints.getMinimalLinearSpeed());
      ASSERT_LE(std::abs(clampedCommandFrame.angularSpeed),userConstraints.getMaximalAbsoluteAngularSpeed());

      romea::OdometryFrame2WD odometryFrame;
      romea::forwardKinematic(parameters,clampedCommandFrame,odometryFrame);

      ASSERT_LE(std::abs(odometryFrame.leftWheelSpeed),parameters.maximalWheelSpeed);
      ASSERT_LE(std::abs(odometryFrame.rightWheelSpeed),parameters.maximalWheelSpeed);

      if(userConstraints.getMaximalLinearSpeed()>=std::numeric_limits<double>::max() &&
         userConstraints.getMinimalLinearSpeed()<=std::numeric_limits<double>::min() &&
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


////-----------------------------------------------------------------------------
//inline void testInverseForward2WD(const romea::SkidSteeringKinematic & kinematic,
//                                  const romea::SkidSteeringOdometry & odometry)
//{


//  for(size_t i=0;i<21;i++)
//  {
//    double linearSpeed=-1+i*0.1;
//    for(size_t j=0;j<21;j++)
//    {

//      double angularSpeed = -0.5+j*0.05;

//      double instantaneousCurvature;
//      auto div = romea::safe_divide(angularSpeed,linearSpeed);

//      if(div)
//      {
//        instantaneousCurvature=*div;
//      }
//      else
//      {
//        instantaneousCurvature=0;
//      }

//      romea::KinematicCommand kinematicCommand;
//      kinematicCommand.linearSpeed =linearSpeed;
//      kinematicCommand.beta =0;
//      kinematicCommand.angularSpeed=angularSpeed;
//      kinematicCommand.instantaneousCurvature = instantaneousCurvature;

//      romea::OdometryFrame2WD odometryFrame = romea::forwardKinematic2WD(odometry.getName(),kinematic,kinematicCommand);
//      romea::KinematicMeasure kinematicMeasure = romea::inverseKinematic2WD(kinematic,odometry,odometryFrame);

//      EXPECT_NEAR(linearSpeed,kinematicMeasure.linearSpeed,0.001);
//      EXPECT_NEAR(0,kinematicMeasure.beta,0.001);
//      EXPECT_NEAR(angularSpeed,kinematicMeasure.angularSpeed,0.001);

//      Eigen::Matrix2d covarianceOdometry = Eigen::Matrix2d::Zero();
//      covarianceOdometry(0,0) = odometry.getWheelSpeedVariance();
//      covarianceOdometry(1,1) = odometry.getWheelSpeedVariance();

//      double T = kinematic.getTrack("track").get();
//      double vl = odometryFrame.leftWheelSpeed;
//      double vr = odometryFrame.rightWheelSpeed;
//      double speed = 0.5*(vl+vr);

//      if(std::abs(speed)>std::numeric_limits<double>::epsilon())
//      {

//        Eigen::MatrixXd J(4,2);
//        J.row(0) << 1/2. , 1/2.;
//        J.row(1) << 0   , 0   ;
//        J.row(2) << 1/T , -1/T;
//        J.row(3) << vl/(T*speed*speed) , -vr/(T*speed*speed);

//        EXPECT_NEAR(0,((J*covarianceOdometry*J.transpose()).array() - kinematicMeasure.covariance.array()).sum(),0.0001);
//      }
//    }
//  }
//}

TEST(testInverseForward2D, NoLimit)
{


  romea::SkidSteeringKinematic::Parameters parameters;
  parameters.track = 0.6;
  parameters.wheelSpeedVariance = 0.1*0.1;
  testInverseForward2WD(parameters,romea::SkidSteeringConstraints());
}

TEST(testInverseForward2D, MechanicalLimits)
{
  romea::SkidSteeringKinematic::Parameters parameters;
  parameters.track = 2.5;
  parameters.maximalWheelSpeed = 1;
  parameters.wheelSpeedVariance = 0.1*0.1;
  testInverseForward2WD(parameters,romea::SkidSteeringConstraints());
}

TEST(testInverseForward2D, UserLimits)
{
  romea::SkidSteeringKinematic::Parameters parameters;
  parameters.track = 2.5;
  parameters.maximalWheelSpeed=1;
  parameters.wheelSpeedVariance = 0.1*0.1;
  testInverseForward2WD(parameters,romea::SkidSteeringConstraints(-0.4,0.8,0.5));
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
