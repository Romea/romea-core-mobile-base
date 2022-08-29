#ifndef romea_OdometryFrame4WS4WD_hpp
#define romea_OdometryFrame4WS4WD_hpp

//stl
#include <ostream>

namespace romea {



struct OdometryFrame4WS4WD
{
  double frontLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;

  double frontRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;

  double rearLeftWheelLinearSpeed;
  double rearLeftWheelSteeringAngle;

  double rearRightWheelLinearSpeed;
  double rearRightWheelSteeringAngle;
};

//struct OdometryFrame4WS4WDa
//{
//  double frontLeftWheelAngularSpeed;
//  double frontLeftWheelSteeringAngle;

//  double frontRightWheelAngularSpeed;
//  double frontRightWheelSteeringAngle;

//  double rearLeftWheelAngularSpeed;
//  double rearLeftWheelSteeringAngle;

//  double rearRightWheelAngularSpeed;
//  double rearRightWheelSteeringAngle;

//};


//std::ostream & operator<<(std::ostream &s, const OdometryFrame4WS4WDl &frame);

}//end romea
#endif
