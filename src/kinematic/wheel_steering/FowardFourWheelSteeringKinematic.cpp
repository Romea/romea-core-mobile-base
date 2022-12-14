// std
#include <cassert>

// romea
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {

//-----------------------------------------------------------------------------
void  forwardKinematic(const FourWheelSteeringKinematic::Parameters & parameters,
                       const TwoAxleSteeringCommand & commandFrame,
                       OdometryFrame4WS4WD &commandOdometryFrame)
{


  const double frontWheelBase = parameters.frontWheelBase;
  const double rearWheelBase = parameters.rearWheelBase;
  const double halfWheelTrack = parameters.wheelTrack/2.;
  const double hubCarrierOffset = parameters.hubCarrierOffset;

  double speed = commandFrame.longitudinalSpeed;

  double tanFrontSteeringAngle = std::tan(commandFrame.frontSteeringAngle);
  double tanRearSteeringAngle = std::tan(commandFrame.rearSteeringAngle);

  double orthogonalInstantaneousCurvature = TwoAxleSteeringKinematic::
      computeInstantaneousCurvature(tanFrontSteeringAngle,
                                    tanRearSteeringAngle,
                                    frontWheelBase,
                                    rearWheelBase);

  double frontLeftWheelAngle  = TwoWheelSteeringKinematic::
      computeLeftWheelSteeringAngle(tanFrontSteeringAngle,
                                    orthogonalInstantaneousCurvature,
                                    halfWheelTrack);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
      computeRightWheelSteeringAngle(tanFrontSteeringAngle,
                                     orthogonalInstantaneousCurvature,
                                     halfWheelTrack);

  double rearLeftWheelAngle  = TwoWheelSteeringKinematic::
      computeLeftWheelSteeringAngle(tanRearSteeringAngle,
                                    orthogonalInstantaneousCurvature,
                                    halfWheelTrack);

  double rearRightWheelAngle = TwoWheelSteeringKinematic::
      computeRightWheelSteeringAngle(tanRearSteeringAngle,
                                     orthogonalInstantaneousCurvature,
                                     halfWheelTrack);

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
      computeLeftWheelLinearSpeed(speed,
                                  tanFrontSteeringAngle,
                                  orthogonalInstantaneousCurvature,
                                  hubCarrierOffset,
                                  halfWheelTrack);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
      computeRightWheelLinearSpeed(speed,
                                   tanFrontSteeringAngle,
                                   orthogonalInstantaneousCurvature,
                                   hubCarrierOffset,
                                   halfWheelTrack);


  double rearLeftWheelSpeed = OneAxleSteeringKinematic::
      computeLeftWheelLinearSpeed(speed,
                                  tanRearSteeringAngle,
                                  orthogonalInstantaneousCurvature,
                                  hubCarrierOffset,
                                  halfWheelTrack);

  double rearRightWheelSpeed = OneAxleSteeringKinematic::
      computeRightWheelLinearSpeed(speed,
                                   tanRearSteeringAngle,
                                   orthogonalInstantaneousCurvature,
                                   hubCarrierOffset,
                                   halfWheelTrack);

  assert(std::abs(frontLeftWheelAngle) <= M_PI_2);
  assert(std::abs(frontRightWheelAngle) <= M_PI_2);
  assert(sign(frontLeftWheelSpeed) == sign(speed));
  assert(sign(frontRightWheelSpeed) == sign(speed));
  assert(sign(rearLeftWheelSpeed) == sign(speed));
  assert(sign(rearRightWheelSpeed) == sign(speed));
  assert(sign(frontLeftWheelAngle) == sign(frontRightWheelAngle));
  assert(sign(rearLeftWheelAngle) == sign(rearRightWheelAngle));


  commandOdometryFrame.frontLeftWheelLinearSpeed  = frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle  = frontLeftWheelAngle;
  commandOdometryFrame.frontRightWheelLinearSpeed  = frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle = frontRightWheelAngle;
  commandOdometryFrame.rearLeftWheelLinearSpeed  = rearLeftWheelSpeed;
  commandOdometryFrame.rearLeftWheelSteeringAngle = rearLeftWheelAngle;
  commandOdometryFrame.rearRightWheelLinearSpeed = rearRightWheelSpeed;
  commandOdometryFrame.rearRightWheelSteeringAngle = rearRightWheelAngle;
}

}  // namespace romea
