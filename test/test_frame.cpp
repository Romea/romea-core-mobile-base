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




//TEST(TestOdometry, testFrame2WD)
//{
//  double leftSpeed = 1;
//  double rightSpeed = 2;

//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("left_wheel_speed",leftSpeed);
//  odometryFrame.set("right_wheel_speed",rightSpeed);

//  romea::OdometryFrame2WD odometryFrame2D;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame2D);
//  EXPECT_FLOAT_EQ(odometryFrame2D.leftWheelSpeed,leftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2D.rightWheelSpeed,rightSpeed);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame2D,odometryFrameBis);
//  double leftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("left_wheel_speed",leftSpeedBis));
//  EXPECT_FLOAT_EQ(leftSpeed,leftSpeedBis);
//  double rightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("right_wheel_speed",rightSpeedBis));
//  EXPECT_FLOAT_EQ(rightSpeed,rightSpeedBis);


//}

//TEST(TestOdometry, testFrame4WD)
//{
//  double frontLeftSpeed = 1;
//  double frontRightSpeed = 2;
//  double rearLeftSpeed = 3;
//  double rearRightSpeed = 4;

//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("front_left_wheel_speed",frontLeftSpeed);
//  odometryFrame.set("front_right_wheel_speed",frontRightSpeed);
//  odometryFrame.set("rear_left_wheel_speed",rearLeftSpeed);
//  odometryFrame.set("rear_right_wheel_speed",rearRightSpeed);

//  romea::OdometryFrame4WD odometryFrame4WD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame4WD);
//  EXPECT_FLOAT_EQ(odometryFrame4WD.frontLeftWheelSpeed,frontLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WD.frontRightWheelSpeed,frontRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WD.rearLeftWheelSpeed,rearLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WD.rearRightWheelSpeed,rearRightSpeed);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame4WD,odometryFrameBis);
//  double frontLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_speed",frontLeftSpeedBis));
//  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
//  double frontRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_speed",frontRightSpeedBis));
//  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
//  double rearLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_left_wheel_speed",rearLeftSpeedBis));
//  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
//  double rearRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_right_wheel_speed",rearRightSpeedBis));
//  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);


//}

//TEST(TestOdometry, testFrame1FAS2RWD)
//{
//  double rearLeftSpeed = 3;
//  double rearRightSpeed = 4;
//  double frontSteeringAngle =5;

//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("rear_left_wheel_speed",rearLeftSpeed);
//  odometryFrame.set("rear_right_wheel_speed",rearRightSpeed);
//  odometryFrame.set("front_axle_steering_angle",frontSteeringAngle);

//  romea::OdometryFrame1FAS2RWD odometryFrame1FAS2RWD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame1FAS2RWD);
//  EXPECT_FLOAT_EQ(odometryFrame1FAS2RWD.rearLeftWheelSpeed,rearLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame1FAS2RWD.rearRightWheelSpeed,rearRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame1FAS2RWD.frontAxleSteeringAngle,frontSteeringAngle);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame1FAS2RWD,odometryFrameBis);
//  double rearLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_left_wheel_speed",rearLeftSpeedBis));
//  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
//  double rearRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_right_wheel_speed",rearRightSpeedBis));
//  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
//  double frontSteeringAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_axle_steering_angle",frontSteeringAngleBis));
//  EXPECT_FLOAT_EQ(frontSteeringAngle,frontSteeringAngleBis);


//}


//TEST(TestOdometry, testFrame1FAS2FWD)
//{
//  double frontLeftSpeed = 1;
//  double frontRightSpeed = 2;
//  double frontSteeringAngle =5;

//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("front_left_wheel_speed",frontLeftSpeed);
//  odometryFrame.set("front_right_wheel_speed",frontRightSpeed);
//  odometryFrame.set("front_axle_steering_angle",frontSteeringAngle);

//  romea::OdometryFrame1FAS2FWD odometryFrame1FAS2FWD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame1FAS2FWD);
//  EXPECT_FLOAT_EQ(odometryFrame1FAS2FWD.frontLeftWheelSpeed,frontLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame1FAS2FWD.frontRightWheelSpeed,frontRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame1FAS2FWD.frontAxleSteeringAngle,frontSteeringAngle);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame1FAS2FWD,odometryFrameBis);
//  double frontLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_speed",frontLeftSpeedBis));
//  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
//  double frontRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_speed",frontRightSpeedBis));
//  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
//  double frontSteeringAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_axle_steering_angle",frontSteeringAngleBis));
//  EXPECT_FLOAT_EQ(frontSteeringAngle,frontSteeringAngleBis);
//}


//TEST(TestOdometry, testFrame2AS4WD)
//{
//  double frontLeftSpeed = 1;
//  double frontRightSpeed = 2;
//  double rearLeftSpeed = 3;
//  double rearRightSpeed = 4;
//  double frontSteeringAngle =5;
//  double rearSteeringAngle =6;

//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("front_left_wheel_speed",frontLeftSpeed);
//  odometryFrame.set("front_right_wheel_speed",frontRightSpeed);
//  odometryFrame.set("rear_left_wheel_speed",rearLeftSpeed);
//  odometryFrame.set("rear_right_wheel_speed",rearRightSpeed);
//  odometryFrame.set("front_axle_steering_angle",frontSteeringAngle);
//  odometryFrame.set("rear_axle_steering_angle",rearSteeringAngle);

//  romea::OdometryFrame2AS4WD odometryFrame2AS4WD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame2AS4WD);
//  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.frontLeftWheelSpeed,frontLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.frontRightWheelSpeed,frontRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.rearLeftWheelSpeed,rearLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.rearRightWheelSpeed,rearRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.frontAxleSteeringAngle,frontSteeringAngle);
//  EXPECT_FLOAT_EQ(odometryFrame2AS4WD.rearAxleSteeringAngle,rearSteeringAngle);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame2AS4WD,odometryFrameBis);
//  double frontLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_speed",frontLeftSpeedBis));
//  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
//  double frontRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_speed",frontRightSpeedBis));
//  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
//  double rearLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_left_wheel_speed",rearLeftSpeedBis));
//  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
//  double rearRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_right_wheel_speed",rearRightSpeedBis));
//  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
//  double frontSteeringAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_axle_steering_angle",frontSteeringAngleBis));
//  EXPECT_FLOAT_EQ(frontSteeringAngle,frontSteeringAngleBis);
//  double rearSteeringAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_axle_steering_angle",rearSteeringAngleBis));
//  EXPECT_FLOAT_EQ(rearSteeringAngle,rearSteeringAngleBis);


//}

//TEST(TestOdometry, testFrame4WS4WD)
//{
//  double frontLeftSpeed = 1;
//  double frontRightSpeed = 2;
//  double rearLeftSpeed = 3;
//  double rearRightSpeed = 4;
//  double frontLeftWheelAngle =5;
//  double frontRightWheelAngle =6;
//  double rearLeftWheelAngle =7;
//  double rearRightWheelAngle =8;


//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("front_left_wheel_speed",frontLeftSpeed);
//  odometryFrame.set("front_right_wheel_speed",frontRightSpeed);
//  odometryFrame.set("rear_left_wheel_speed",rearLeftSpeed);
//  odometryFrame.set("rear_right_wheel_speed",rearRightSpeed);
//  odometryFrame.set("front_left_wheel_angle",frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",frontRightWheelAngle);
//  odometryFrame.set("rear_left_wheel_angle",rearLeftWheelAngle);
//  odometryFrame.set("rear_right_wheel_angle",rearRightWheelAngle);

//  romea::OdometryFrame4WS4WD odometryFrame4WS4WD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame4WS4WD);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontLeftWheelSpeed,frontLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontRightWheelSpeed,frontRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearLeftWheelSpeed,rearLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearRightWheelSpeed,rearRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontLeftWheelAngle,frontLeftWheelAngle);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.frontRightWheelAngle,frontRightWheelAngle);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearLeftWheelAngle,rearLeftWheelAngle);
//  EXPECT_FLOAT_EQ(odometryFrame4WS4WD.rearRightWheelAngle,rearRightWheelAngle);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame4WS4WD,odometryFrameBis);
//  double frontLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_speed",frontLeftSpeedBis));
//  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
//  double frontRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_speed",frontRightSpeedBis));
//  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
//  double rearLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_left_wheel_speed",rearLeftSpeedBis));
//  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
//  double rearRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_right_wheel_speed",rearRightSpeedBis));
//  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
//  double frontLeftWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_angle",frontLeftWheelAngleBis));
//  EXPECT_FLOAT_EQ(frontLeftWheelAngle,frontLeftWheelAngleBis);
//  double frontRightWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_angle",frontRightWheelAngleBis));
//  EXPECT_FLOAT_EQ(frontRightWheelAngle,frontRightWheelAngleBis);
//  double rearLeftWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_left_wheel_angle",rearLeftWheelAngleBis));
//  EXPECT_FLOAT_EQ(rearLeftWheelAngle,rearLeftWheelAngleBis);
//  double rearRightWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_right_wheel_angle",rearRightWheelAngleBis));
//  EXPECT_FLOAT_EQ(rearRightWheelAngle,rearRightWheelAngleBis);


//}

//TEST(TestOdometry, OdometryFrame2FWS2RWD)
//{
//  double rearLeftSpeed = 3;
//  double rearRightSpeed = 4;
//  double frontLeftWheelAngle =5;
//  double frontRightWheelAngle =6;


//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("rear_left_wheel_speed",rearLeftSpeed);
//  odometryFrame.set("rear_right_wheel_speed",rearRightSpeed);
//  odometryFrame.set("front_left_wheel_angle",frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",frontRightWheelAngle);

//  romea::OdometryFrame2FWS2RWD odometryFrame2FWS2RWD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame2FWS2RWD);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.rearLeftWheelSpeed,rearLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.rearRightWheelSpeed,rearRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.frontLeftWheelAngle,frontLeftWheelAngle);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2RWD.frontRightWheelAngle,frontRightWheelAngle);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame2FWS2RWD,odometryFrameBis);
//  double rearLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_left_wheel_speed",rearLeftSpeedBis));
//  EXPECT_FLOAT_EQ(rearLeftSpeed,rearLeftSpeedBis);
//  double rearRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("rear_right_wheel_speed",rearRightSpeedBis));
//  EXPECT_FLOAT_EQ(rearRightSpeed,rearRightSpeedBis);
//  double frontLeftWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_angle",frontLeftWheelAngleBis));
//  EXPECT_FLOAT_EQ(frontLeftWheelAngle,frontLeftWheelAngleBis);
//  double frontRightWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_angle",frontRightWheelAngleBis));
//  EXPECT_FLOAT_EQ(frontRightWheelAngle,frontRightWheelAngleBis);
//}

//TEST(TestOdometry, testFrame2FWS2FWD)
//{
//  double frontLeftSpeed = 1;
//  double frontRightSpeed = 2;
//  double frontLeftWheelAngle =5;
//  double frontRightWheelAngle =6;


//  romea::OdometryFrame odometryFrame;
//  odometryFrame.set("front_left_wheel_speed",frontLeftSpeed);
//  odometryFrame.set("front_right_wheel_speed",frontRightSpeed);
//  odometryFrame.set("front_left_wheel_angle",frontLeftWheelAngle);
//  odometryFrame.set("front_right_wheel_angle",frontRightWheelAngle);

//  romea::OdometryFrame2FWS2FWD odometryFrame2FWS2FWD;
//  romea::fromOdometryFrame(odometryFrame,odometryFrame2FWS2FWD);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontLeftWheelSpeed,frontLeftSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontRightWheelSpeed,frontRightSpeed);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontLeftWheelAngle,frontLeftWheelAngle);
//  EXPECT_FLOAT_EQ(odometryFrame2FWS2FWD.frontRightWheelAngle,frontRightWheelAngle);

//  romea::OdometryFrame odometryFrameBis;
//  romea::toOdometryFrame(odometryFrame2FWS2FWD,odometryFrameBis);
//  double frontLeftSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_speed",frontLeftSpeedBis));
//  EXPECT_FLOAT_EQ(frontLeftSpeed,frontLeftSpeedBis);
//  double frontRightSpeedBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_wheel_speed",frontRightSpeedBis));
//  EXPECT_FLOAT_EQ(frontRightSpeed,frontRightSpeedBis);
//  double frontLeftWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_left_wheel_angle",frontLeftWheelAngleBis));
//  EXPECT_FLOAT_EQ(frontLeftWheelAngle,frontLeftWheelAngleBis);
//  double frontRightWheelAngleBis;
//  EXPECT_TRUE(odometryFrameBis.get("front_right_whel_angle",frontRightWheelAngleBis));
//  EXPECT_FLOAT_EQ(frontRightWheelAngle,frontRightWheelAngleBis);

//}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
