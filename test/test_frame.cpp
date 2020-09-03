// gtest
#include <gtest/gtest.h>

#include "romea_odo/odometry/OdometryFrame.hpp"
#include "romea_odo/odometry/OdometryFrame2WD.hpp"
#include "romea_odo/odometry/OdometryFrame4WD.hpp"
#include "romea_odo/odometry/OdometryFrame2AS4WD.hpp"
#include "romea_odo/odometry/OdometryFrame1FAS2FWD.hpp"
#include "romea_odo/odometry/OdometryFrame1FAS2RWD.hpp"
#include "romea_odo/odometry/OdometryFrame4WS4WD.hpp"
#include "romea_odo/odometry/OdometryFrame2FWS2FWD.hpp"
#include "romea_odo/odometry/OdometryFrame2FWS2RWD.hpp"




TEST(TestOdometry, testFrame2WD)
{
  double leftSpeed = 1;
  double rightSpeed = 2;

  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("left_wheel",leftSpeed);
  odometryFrame.setWheelSpeed("right_wheel",rightSpeed);

  romea::OdometryFrame2WD odometryFrame2D;
  romea::fromOdometryFrame(odometryFrame,odometryFrame2D);
  EXPECT_FLOAT_EQ(odometryFrame2D.leftWheelSpeed,leftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2D.rightWheelSpeed,rightSpeed);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame2D,odometryFrameBis);
  double leftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("left_wheel",leftSpeedBis));
  EXPECT_FLOAT_EQ(leftSpeed,leftSpeedBis);
  double rightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("right_wheel",rightSpeedBis));
  EXPECT_FLOAT_EQ(rightSpeed,rightSpeedBis);


}

TEST(TestOdometry, testFrame4WD)
{
  double frontLeftSpeed = 1;
  double frontRightSpeed = 2;
  double rearLeftSpeed = 3;
  double rearRightSpeed = 4;

  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("front_left_wheel",frontLeftSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",frontRightSpeed);
  odometryFrame.setWheelSpeed("rear_left_wheel",rearLeftSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",rearRightSpeed);

  romea::OdometryFrame4WD odometryFrame4WD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame4WD);
  EXPECT_FLOAT_EQ(odometryFrame4WD.frontLeftWheelSpeed,frontLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WD.frontRightWheelSpeed,frontRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WD.rearLeftWheelSpeed,rearLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WD.rearRightWheelSpeed,rearRightSpeed);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame4WD,odometryFrameBis);
  double frontLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_left_wheel",frontLeftSpeedBis));
  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
  double frontRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_right_wheel",frontRightSpeedBis));
  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
  double rearLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_left_wheel",rearLeftSpeedBis));
  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
  double rearRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_right_wheel",rearRightSpeedBis));
  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);


}

TEST(TestOdometry, testFrame1FAS2RWD)
{
  double rearLeftSpeed = 3;
  double rearRightSpeed = 4;
  double frontSteeringAngle =5;

  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("rear_left_wheel",rearLeftSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",rearRightSpeed);
  odometryFrame.setSteeringAngle("front_steering",frontSteeringAngle);

  romea::OdometryFrame1FAS2RWD odometryFrame1FAS2RWD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame1FAS2RWD);
  EXPECT_FLOAT_EQ(odometryFrame1FAS2RWD.rearLeftWheelSpeed,rearLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame1FAS2RWD.rearRightWheelSpeed,rearRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame1FAS2RWD.frontAxleSteeringAngle,frontSteeringAngle);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame1FAS2RWD,odometryFrameBis);
  double rearLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_left_wheel",rearLeftSpeedBis));
  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
  double rearRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_right_wheel",rearRightSpeedBis));
  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
  double frontSteeringAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_steering",frontSteeringAngleBis));
  EXPECT_FLOAT_EQ(frontSteeringAngle,frontSteeringAngleBis);


}


TEST(TestOdometry, testFrame1FAS2FWD)
{
  double frontLeftSpeed = 1;
  double frontRightSpeed = 2;
  double frontSteeringAngle =5;

  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("front_left_wheel",frontLeftSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",frontRightSpeed);
  odometryFrame.setSteeringAngle("front_steering",frontSteeringAngle);

  romea::OdometryFrame1FAS2FWD odometryFrame1FAS2FWD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame1FAS2FWD);
  EXPECT_FLOAT_EQ(odometryFrame1FAS2FWD.frontLeftWheelSpeed,frontLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame1FAS2FWD.frontRightWheelSpeed,frontRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame1FAS2FWD.frontAxleSteeringAngle,frontSteeringAngle);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame1FAS2FWD,odometryFrameBis);
  double frontLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_left_wheel",frontLeftSpeedBis));
  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
  double frontRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_right_wheel",frontRightSpeedBis));
  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
  double frontSteeringAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_steering",frontSteeringAngleBis));
  EXPECT_FLOAT_EQ(frontSteeringAngle,frontSteeringAngleBis);
}


TEST(TestOdometry, testFrame2AS4WD)
{
  double frontLeftSpeed = 1;
  double frontRightSpeed = 2;
  double rearLeftSpeed = 3;
  double rearRightSpeed = 4;
  double frontSteeringAngle =5;
  double rearSteeringAngle =6;

  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("front_left_wheel",frontLeftSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",frontRightSpeed);
  odometryFrame.setWheelSpeed("rear_left_wheel",rearLeftSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",rearRightSpeed);
  odometryFrame.setSteeringAngle("front_steering",frontSteeringAngle);
  odometryFrame.setSteeringAngle("rear_steering",rearSteeringAngle);

  romea::OdometryFrame2AS4WD odometryFrame2AS4WD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame2AS4WD);
  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.frontLeftWheelSpeed,frontLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.frontRightWheelSpeed,frontRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.rearLeftWheelSpeed,rearLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.rearRightWheelSpeed,rearRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.frontAxleSteeringAngle,frontSteeringAngle);
  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.rearAxleSteeringAngle,rearSteeringAngle);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame2AS4WD,odometryFrameBis);
  double frontLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_left_wheel",frontLeftSpeedBis));
  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
  double frontRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_right_wheel",frontRightSpeedBis));
  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
  double rearLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_left_wheel",rearLeftSpeedBis));
  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
  double rearRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_right_wheel",rearRightSpeedBis));
  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
  double frontSteeringAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_steering",frontSteeringAngleBis));
  EXPECT_FLOAT_EQ(frontSteeringAngle,frontSteeringAngleBis);
  double rearSteeringAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("rear_steering",rearSteeringAngleBis));
  EXPECT_FLOAT_EQ(rearSteeringAngle,rearSteeringAngleBis);


}

TEST(TestOdometry, testFrame4WS4WD)
{
  double frontLeftSpeed = 1;
  double frontRightSpeed = 2;
  double rearLeftSpeed = 3;
  double rearRightSpeed = 4;
  double frontLeftWheelAngle =5;
  double frontRightWheelAngle =6;
  double rearLeftWheelAngle =7;
  double rearRightWheelAngle =8;


  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("front_left_wheel",frontLeftSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",frontRightSpeed);
  odometryFrame.setWheelSpeed("rear_left_wheel",rearLeftSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",rearRightSpeed);
  odometryFrame.setSteeringAngle("front_left_steering",frontLeftWheelAngle);
  odometryFrame.setSteeringAngle("front_right_steering",frontRightWheelAngle);
  odometryFrame.setSteeringAngle("rear_left_steering",rearLeftWheelAngle);
  odometryFrame.setSteeringAngle("rear_right_steering",rearRightWheelAngle);

  romea::OdometryFrame4WS4WD odometryFrame4WS4WD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame4WS4WD);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontLeftWheelSpeed,frontLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontRightWheelSpeed,frontRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearLeftWheelSpeed,rearLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearRightWheelSpeed,rearRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontLeftWheelAngle,frontLeftWheelAngle);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontRightWheelAngle,frontRightWheelAngle);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearLeftWheelAngle,rearLeftWheelAngle);
  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearRightWheelAngle,rearRightWheelAngle);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame4WS4WD,odometryFrameBis);
  double frontLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_left_wheel",frontLeftSpeedBis));
  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
  double frontRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_right_wheel",frontRightSpeedBis));
  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
  double rearLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_left_wheel",rearLeftSpeedBis));
  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
  double rearRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_right_wheel",rearRightSpeedBis));
  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
  double frontLeftWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_left_steering",frontLeftWheelAngleBis));
  EXPECT_FLOAT_EQ(frontLeftWheelAngle,frontLeftWheelAngleBis);
  double frontRightWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_right_steering",frontRightWheelAngleBis));
  EXPECT_FLOAT_EQ(frontRightWheelAngle,frontRightWheelAngleBis);
  double rearLeftWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("rear_left_steering",rearLeftWheelAngleBis));
  EXPECT_FLOAT_EQ(rearLeftWheelAngle,rearLeftWheelAngleBis);
  double rearRightWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("rear_right_steering",rearRightWheelAngleBis));
  EXPECT_FLOAT_EQ(rearRightWheelAngle,rearRightWheelAngleBis);


}

TEST(TestOdometry, OdometryFrame2FWS2RWD)
{
  double rearLeftSpeed = 3;
  double rearRightSpeed = 4;
  double frontLeftWheelAngle =5;
  double frontRightWheelAngle =6;


  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("rear_left_wheel",rearLeftSpeed);
  odometryFrame.setWheelSpeed("rear_right_wheel",rearRightSpeed);
  odometryFrame.setSteeringAngle("front_left_steering",frontLeftWheelAngle);
  odometryFrame.setSteeringAngle("front_right_steering",frontRightWheelAngle);

  romea::OdometryFrame2FWS2RWD odometryFrame2FWS2RWD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame2FWS2RWD);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.rearLeftWheelSpeed,rearLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.rearRightWheelSpeed,rearRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.frontLeftWheelAngle,frontLeftWheelAngle);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.frontRightWheelAngle,frontRightWheelAngle);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame2FWS2RWD,odometryFrameBis);
  double rearLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_left_wheel",rearLeftSpeedBis));
  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
  double rearRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("rear_right_wheel",rearRightSpeedBis));
  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
  double frontLeftWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_left_steering",frontLeftWheelAngleBis));
  EXPECT_FLOAT_EQ(frontLeftWheelAngle,frontLeftWheelAngleBis);
  double frontRightWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_right_steering",frontRightWheelAngleBis));
  EXPECT_FLOAT_EQ(frontRightWheelAngle,frontRightWheelAngleBis);
}

TEST(TestOdometry, testFrame2FWS2FWD)
{
  double frontLeftSpeed = 1;
  double frontRightSpeed = 2;
  double frontLeftWheelAngle =5;
  double frontRightWheelAngle =6;


  romea::OdometryFrame odometryFrame;
  odometryFrame.setWheelSpeed("front_left_wheel",frontLeftSpeed);
  odometryFrame.setWheelSpeed("front_right_wheel",frontRightSpeed);
  odometryFrame.setSteeringAngle("front_left_steering",frontLeftWheelAngle);
  odometryFrame.setSteeringAngle("front_right_steering",frontRightWheelAngle);

  romea::OdometryFrame2FWS2FWD odometryFrame2FWS2FWD;
  romea::fromOdometryFrame(odometryFrame,odometryFrame2FWS2FWD);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontLeftWheelSpeed,frontLeftSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontRightWheelSpeed,frontRightSpeed);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontLeftWheelAngle,frontLeftWheelAngle);
  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontRightWheelAngle,frontRightWheelAngle);

  romea::OdometryFrame odometryFrameBis;
  romea::toOdometryFrame(odometryFrame2FWS2FWD,odometryFrameBis);
  double frontLeftSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_left_wheel",frontLeftSpeedBis));
  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
  double frontRightSpeedBis;
  EXPECT_TRUE(odometryFrameBis.getWheelSpeed("front_right_wheel",frontRightSpeedBis));
  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
  double frontLeftWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_left_steering",frontLeftWheelAngleBis));
  EXPECT_FLOAT_EQ(frontLeftWheelAngle,frontLeftWheelAngleBis);
  double frontRightWheelAngleBis;
  EXPECT_TRUE(odometryFrameBis.getSteeringAngle("front_right_steering",frontRightWheelAngleBis));
  EXPECT_FLOAT_EQ(frontRightWheelAngle,frontRightWheelAngleBis);

}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
