//romea
#include "romea_odo/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>

#include <iostream>

namespace romea {


//-----------------------------------------------------------------------------
void forwardKinematic(const TwoAxleSteeringKinematic::Parameters &parameters,
                      const TwoAxleSteeringCommand & commandFrame,
                      OdometryFrame2AS4WD & odometryCommandFrame)
{

  double wheelbase = parameters.frontWheelBase + parameters.rearWheelBase;

  const double & linearSpeed = commandFrame.longitudinalSpeed;
  double frontSteeringAngle = commandFrame.frontSteeringAngle;
  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  const double frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  const double frontHalfTrack = parameters.frontTrack/2;

  double rearSteeringAngle = commandFrame.rearSteeringAngle;
  double tanRearSteeringAngle = std::tan(rearSteeringAngle);
  const double rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double rearHalfTrack = parameters.rearTrack/2;

  double instantaneousCurvature = (tanFrontSteeringAngle - tanRearSteeringAngle)/wheelbase;

  double frontLeftWheelSpeed=OneAxleSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
                                                                             tanFrontSteeringAngle,
                                                                             instantaneousCurvature,
                                                                             frontHubCarrierOffset,
                                                                             frontHalfTrack);

  double frontRightWheelSpeed=OneAxleSteeringKinematic::computeRightWheelSpeed(linearSpeed,
                                                                               tanFrontSteeringAngle,
                                                                               instantaneousCurvature,
                                                                               frontHubCarrierOffset,
                                                                               frontHalfTrack);


  double rearLeftWheelSpeed=OneAxleSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
                                                                            tanRearSteeringAngle,
                                                                            instantaneousCurvature,
                                                                            rearHubCarrierOffset,
                                                                            rearHalfTrack);

  double rearRightWheelSpeed=OneAxleSteeringKinematic::computeRightWheelSpeed(linearSpeed,
                                                                              tanRearSteeringAngle,
                                                                              instantaneousCurvature,
                                                                              rearHubCarrierOffset,
                                                                              rearHalfTrack );

  assert(sign(frontLeftWheelSpeed)==sign(linearSpeed));
  assert(sign(frontRightWheelSpeed)==sign(linearSpeed));
  assert(sign(rearLeftWheelSpeed)==sign(linearSpeed));
  assert(sign(rearRightWheelSpeed)==sign(linearSpeed));

  odometryCommandFrame.frontAxleSteeringAngle=frontSteeringAngle;
  odometryCommandFrame.frontLeftWheelSpeed=frontLeftWheelSpeed;
  odometryCommandFrame.frontRightWheelSpeed=frontRightWheelSpeed;
  odometryCommandFrame.rearAxleSteeringAngle=rearSteeringAngle;
  odometryCommandFrame.rearLeftWheelSpeed=rearLeftWheelSpeed;
  odometryCommandFrame.rearRightWheelSpeed=rearRightWheelSpeed;

}

////-----------------------------------------------------------------------------
//OdometryFrame2AS4WD forwardKinematic2AS4WD(const std::string &emitterName,
//                                           const TwoAxleSteeringKinematic & kinematic,
//                                           const KinematicCommand & commandFrame)
//{

//  const double frontWheelBase = kinematic.getWheelBase("front_wheelbase").get();
//  const double rearWheelBase = kinematic.getWheelBase("rear_wheelbase").get();

//  return forwardKinematic2AS4WD(emitterName,
//                                kinematic,
//                                toTwoAxleSteeringCommand(commandFrame,
//                                                         frontWheelBase,
//                                                         rearWheelBase));
//}

}
