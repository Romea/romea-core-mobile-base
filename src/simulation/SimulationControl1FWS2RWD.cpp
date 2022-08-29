#include "romea_core_mobile_base/simulation/SimulationControl1FWS2RWD.hpp"
#include <cmath>

namespace romea {

//-----------------------------------------------------------------------------
SimulationCommand1FWS2RWD toSimulationCommand1FWS2RWD(const double & frontWheelRadius,
                                                      const double & rearWheelRadius,
                                                      const HardwareCommand1FWS2RWD & hardwareCommand)
{
  double linearSpeed = rearWheelRadius *
      (hardwareCommand.rearLeftWheelSetPoint +
       hardwareCommand.rearRightWheelSetPoint);

  double frontWheelSetPoint = linearSpeed/frontWheelRadius*
      std::cos(hardwareCommand.frontWheelSteeringAngle);

  return {hardwareCommand.frontWheelSteeringAngle,
        frontWheelSetPoint,
        hardwareCommand.rearLeftWheelSetPoint,
        hardwareCommand.rearRightWheelSetPoint};

}

//-----------------------------------------------------------------------------
HardwareState1FWS2RWD toHardwareState1FWS2RWD(const SimulationState1FWS2RWD & simulationState)
{
  return {simulationState.frontWheelSteeringAngle,
        simulationState.rearLeftWHeelSpinMotion,
        simulationState.rearRightWHeelSpinMotion};
}


}//end romea
