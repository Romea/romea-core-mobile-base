#include "romea_core_mobile_base/simulation/SimulationControl1FAS4WD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand1FAS4WD toSimulationCommand1FAS4WD(const HardwareCommand1FAS4WD & hardwareCommand,
                                                    const double & frontLeftWheelSteeringAngle,
                                                    const double & frontRightWheelSteeringAngle)
{
  return {hardwareCommand.frontAxleSteeringAngle,
        frontLeftWheelSteeringAngle,
        frontRightWheelSteeringAngle,
        hardwareCommand.frontLeftWheelSpinningSetPoint,
        hardwareCommand.frontRightWheelSpinningSetPoint,
        hardwareCommand.rearLeftWheelSpinningSetPoint,
        hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand1FAS4WD toSimulationCommand1FAS2RWD(const double & wheelbase,
                                                     const double & frontTrack,
                                                     const HardwareCommand1FAS4WD & hardwareCommand)
{

  double  tanAxleSteeringAngle=
      std::tan(hardwareCommand.frontAxleSteeringAngle);

  double intantaneousCurvature= OneAxleSteeringKinematic::
      computeInstantaneousCurvature(tanAxleSteeringAngle,wheelbase);


  double frontLeftWheelSteeringAngle = TwoWheelSteeringKinematic::
      computeLeftWheelSteeringAngle(tanAxleSteeringAngle,
                                    intantaneousCurvature,
                                    frontTrack/2.);

  double frontRightWheelSteeringAngle = TwoWheelSteeringKinematic::
      computeRightWheelSteeringAngle(tanAxleSteeringAngle,
                                     intantaneousCurvature,
                                     frontTrack/2.);

  return toSimulationCommand1FAS4WD(hardwareCommand,
                                    frontLeftWheelSteeringAngle,
                                    frontRightWheelSteeringAngle);
}

//-----------------------------------------------------------------------------
HardwareState1FAS4WD toHardwareState1FAS4WD(const SimulationState1FAS4WD & simulationState,
                                            const double frontAxleSteeringAngle)
{
  return { frontAxleSteeringAngle,
        simulationState.frontLeftWheelSpinningMotion,
        simulationState.frontRightWheelSpinningMotion,
        simulationState.rearLeftWheelSpinningMotion,
        simulationState.rearRightWheelSpinningMotion};

}

//-----------------------------------------------------------------------------
HardwareState1FAS4WD toHardwareState1FAS4WD(const double & wheelbase,
                                            const double & frontTrack,
                                            const SimulationState1FAS4WD & simulationState)
{

  double frontAxleSteeringAngle = TwoWheelSteeringKinematic::
      computeSteeringAngle(simulationState.frontLeftWheelSteeringAngle,
                           simulationState.frontRightWheelSteeringAngle,
                           wheelbase,frontTrack);

  return toHardwareState1FAS4WD(simulationState,
                                frontAxleSteeringAngle);
}

}
