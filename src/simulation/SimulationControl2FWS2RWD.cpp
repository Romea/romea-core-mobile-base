#include "romea_core_mobile_base/simulation/SimulationControl2FWS2RWD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2FWS2RWD toSimulationCommand2FWS2RWD(const HardwareCommand2FWS2RWD & hardwareCommand,
                                                      const double & frontLeftWheelSpinningSetPoint,
                                                      const double & frontRightWheelSpinningSetPoint)
{
  return {hardwareCommand.frontLeftWheelSteeringAngle,
        hardwareCommand.frontRightWheelSteeringAngle,
        frontLeftWheelSpinningSetPoint,
        frontRightWheelSpinningSetPoint,
        hardwareCommand.rearLeftWheelSpinningSetPoint,
        hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand2FWS2RWD toSimulationCommand2FWS2RWD(const double & wheelbase,
                                                      const double & frontTrack,
                                                      const double & frontHubCarrierOffset,
                                                      const double & frontWheelRadius,
                                                      const double & rearWheelRadius,
                                                      const HardwareCommand2FWS2RWD & hardwareCommand)
{

  const double & rearLeftWheelLinearSpeed =
      hardwareCommand.rearLeftWheelSpinningSetPoint*rearWheelRadius;

  const double & rearRightWheelLinearSpeed =
      hardwareCommand.rearRightWheelSpinningSetPoint*rearWheelRadius;

  const double & frontLeftWheelSteeringAngle =
      hardwareCommand.frontLeftWheelSteeringAngle;

  const double & frontRightWheelSteeringAngle =
      hardwareCommand.frontRightWheelSteeringAngle;

  double linearSpeed = (rearLeftWheelLinearSpeed+rearRightWheelLinearSpeed)/2.0;

  double instantaneousCurvature = TwoWheelSteeringKinematic::
      computeInstantaneousCurvature(frontLeftWheelSteeringAngle,
                                    frontRightWheelSteeringAngle,
                                    wheelbase,frontTrack);

  double frontFeftWheelLinearSpeed = TwoWheelSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed,
                                  instantaneousCurvature*wheelbase,
                                  instantaneousCurvature,
                                  frontHubCarrierOffset,
                                  frontTrack/2.0);

  double frontRightWheelLinearSpeed= TwoWheelSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed,
                                   instantaneousCurvature*wheelbase,
                                   instantaneousCurvature,
                                   frontHubCarrierOffset,
                                   frontTrack/2.0);


  return toSimulationCommand2FWS2RWD(hardwareCommand,
                                     frontFeftWheelLinearSpeed/frontWheelRadius,
                                     frontRightWheelLinearSpeed/frontWheelRadius);
}

//-----------------------------------------------------------------------------
HardwareState2FWS2RWD toHardwareState2FWS2RWD(const SimulationState2FWS2RWD & simulationState)
{
  return {simulationState.frontLeftWheelSteeringAngle,
        simulationState.frontRightWheelSteeringAngle,
        simulationState.rearLeftWheelSpinningMotion,
        simulationState.rearRightWheelSpinningMotion};
}

}
