//romea
#include "romea_core_odo/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

//std
#include <cmath>

namespace romea {


//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2WD & odometryCommandFrame)
{

  const double track = parameters.wheelTrack;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  odometryCommandFrame.leftWheelSpeed = SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,angularSpeed,track);
  odometryCommandFrame.rightWheelSpeed = SkidSteeringKinematic::computeRightWheelSpeed(linearSpeed,angularSpeed,track);

}

//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand & commandFrame,
                      OdometryFrame4WD & odometryCommandFrame)
{
  const double track = parameters.wheelTrack;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  double leftWheelSpeed = SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,angularSpeed,track);
  double rightWheelSpeed = SkidSteeringKinematic::computeRightWheelSpeed(linearSpeed,angularSpeed,track);

  odometryCommandFrame.frontLeftWheelSpeed = leftWheelSpeed;
  odometryCommandFrame.frontRightWheelSpeed = rightWheelSpeed;
  odometryCommandFrame.rearLeftWheelSpeed = leftWheelSpeed;
  odometryCommandFrame.rearRightWheelSpeed = rightWheelSpeed;
}

////--------------------------------------------------------------------------
//void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
//                      const SkidSteeringCommand &commandFrame,
//                      const OdometryFrame2WD & startOdometryFrame,
//                      OdometryFrame2WD & odometryCommandFrame)
//{
//    const double track = parameters.track;
//    const double & linearSpeed = commandFrame.longitudinalSpeed;
//    const double & angularSpeed = commandFrame.angularSpeed;

//   double
//   double commandLeftWheelSpeed= SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,angularSpeed,track)
//}


}//end romea
