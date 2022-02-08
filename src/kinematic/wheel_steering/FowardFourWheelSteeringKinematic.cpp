//romea
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>
#include <iostream>

namespace romea {

//-----------------------------------------------------------------------------
void  forwardKinematic(const FourWheelSteeringKinematic::Parameters & parameters,
                       const TwoAxleSteeringCommand & commandFrame,
                       OdometryFrame4WS4WD & commandOdometryFrame)
{


  const double frontWheelBase = parameters.frontWheelBase;
  const double rearWheelBase = parameters.rearWheelBase;
  const double halfWheelTrack = parameters.wheelTrack/2.;
  const double hubCarrierOffset = parameters.hubCarrierOffset;

  double speed = commandFrame.longitudinalSpeed;

  double tanFrontSteeringAngle= std::tan(commandFrame.frontSteeringAngle);
  double tanRearSteeringAngle= std::tan(commandFrame.rearSteeringAngle);

  double orthogonalInstantaneousCurvature=
      TwoAxleSteeringKinematic::computeInstantaneousCurvature(tanFrontSteeringAngle,
                                                              tanRearSteeringAngle,
                                                              frontWheelBase,
                                                              rearWheelBase);

  double frontLeftWheelAngle  =
      TwoWheelSteeringKinematic::computeLeftWheelAngle(tanFrontSteeringAngle,
                                                       orthogonalInstantaneousCurvature,
                                                       halfWheelTrack);

  double frontRightWheelAngle =
      TwoWheelSteeringKinematic::computeRightWheelAngle(tanFrontSteeringAngle,
                                                        orthogonalInstantaneousCurvature,
                                                        halfWheelTrack);

  double rearLeftWheelAngle  =
      TwoWheelSteeringKinematic::computeLeftWheelAngle(tanRearSteeringAngle,
                                                       orthogonalInstantaneousCurvature,
                                                       halfWheelTrack);

  double rearRightWheelAngle =
      TwoWheelSteeringKinematic::computeRightWheelAngle(tanRearSteeringAngle,
                                                        orthogonalInstantaneousCurvature,
                                                        halfWheelTrack);



  double frontLeftWheelSpeed =
      OneAxleSteeringKinematic::computeLeftWheelSpeed(speed,
                                                      tanFrontSteeringAngle,
                                                      orthogonalInstantaneousCurvature,
                                                      hubCarrierOffset,
                                                      halfWheelTrack);

  double frontRightWheelSpeed =
      OneAxleSteeringKinematic::computeRightWheelSpeed(speed,
                                                       tanFrontSteeringAngle,
                                                       orthogonalInstantaneousCurvature,
                                                       hubCarrierOffset,
                                                       halfWheelTrack);


  double rearLeftWheelSpeed =
      OneAxleSteeringKinematic::computeLeftWheelSpeed(speed,
                                                      tanRearSteeringAngle,
                                                      orthogonalInstantaneousCurvature,
                                                      hubCarrierOffset,
                                                      halfWheelTrack);

  double rearRightWheelSpeed =
      OneAxleSteeringKinematic::computeRightWheelSpeed(speed,
                                                       tanRearSteeringAngle,
                                                       orthogonalInstantaneousCurvature,
                                                       hubCarrierOffset,
                                                       halfWheelTrack);

  assert(std::abs(frontLeftWheelAngle)<=M_PI_2);
  assert(std::abs(frontRightWheelAngle)<=M_PI_2);
  assert(sign(frontLeftWheelSpeed)==sign(speed));
  assert(sign(frontRightWheelSpeed)==sign(speed));
  assert(sign(rearLeftWheelSpeed)==sign(speed));
  assert(sign(rearRightWheelSpeed)==sign(speed));
  assert(sign(frontLeftWheelAngle)==sign(frontRightWheelAngle));
  assert(sign(rearLeftWheelAngle)==sign(rearRightWheelAngle));


  commandOdometryFrame.frontLeftWheelSpeed =frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelAngle =frontLeftWheelAngle;
  commandOdometryFrame.frontRightWheelSpeed =frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelAngle =frontRightWheelAngle;
  commandOdometryFrame.rearLeftWheelSpeed =rearLeftWheelSpeed;
  commandOdometryFrame.rearLeftWheelAngle =rearLeftWheelAngle;
  commandOdometryFrame.rearRightWheelSpeed =rearRightWheelSpeed;
  commandOdometryFrame.rearRightWheelAngle =rearRightWheelAngle;
}

}
