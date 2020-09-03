//romea
#include "romea_odo/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

//std
#include <cmath>

namespace romea {


//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2WD & odometryCommandFrame)
{

  const double track = parameters.track;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  odometryCommandFrame.leftWheelSpeed = SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,angularSpeed,track);
  odometryCommandFrame.rightWheelSpeed = SkidSteeringKinematic::computeRightWheelSpeed(linearSpeed,angularSpeed,track);

}

////--------------------------------------------------------------------------
//OdometryFrame2WD forwardKinematic2WD(const std::string &emitterName,
//                                     const SkidSteeringKinematic & kinematic,
//                                     const KinematicCommand & commandFrame)
//{

//  return forwardKinematic2WD(emitterName,kinematic,toSkidSteeringCommand(commandFrame));
//}

//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand & commandFrame,
                      OdometryFrame4WD & odometryCommandFrame)
{
  const double track = parameters.track;
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
//OdometryFrame4WD forwardKinematic4WD(const std::string &emitterName,
//                                     const SkidSteeringKinematic & kinematic,
//                                     const KinematicCommand & commandFrame)
//{
//  return forwardKinematic4WD(emitterName,kinematic,toSkidSteeringCommand(commandFrame));
//}



}//end romea
