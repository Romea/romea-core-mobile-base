#include "romea_core_mobile_base/kinematic/omni_steering/InverseMecanumWheelSteeringKinematic.hpp"

namespace romea
{

void inverseKinematic(const MecanumWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame4WD &odometryFrame,
                      OmniSteeringMeasure & omniSteeringMeasure)
{
  const double halfTrack = parameters.wheelTrack/2.;
  const double halfWheebase = parameters.wheelbase/2;

  omniSteeringMeasure.longitudinalSpeed = MecanumWheelSteeringKinematic::
      computeLongitudinalSpeed(odometryFrame.frontLeftWheelLinearSpeed,
                               odometryFrame.frontRightWheelLinearSpeed,
                               odometryFrame.rearLeftWheelLinearSpeed,
                               odometryFrame.rearRightWheelLinearSpeed);

  omniSteeringMeasure.lateralSpeed = MecanumWheelSteeringKinematic::
      computeLateralSpeed(odometryFrame.frontRightWheelLinearSpeed,
                          odometryFrame.rearLeftWheelLinearSpeed,
                          odometryFrame.frontLeftWheelLinearSpeed,
                          odometryFrame.rearRightWheelLinearSpeed);

  omniSteeringMeasure.angularSpeed =
      MecanumWheelSteeringKinematic::
      computeAngularSpeed(odometryFrame.frontRightWheelLinearSpeed,
                          odometryFrame.rearLeftWheelLinearSpeed,
                          odometryFrame.frontLeftWheelLinearSpeed,
                          odometryFrame.rearRightWheelLinearSpeed,
                          halfWheebase,
                          halfTrack);


  Eigen::MatrixXd J = Eigen::MatrixXd::Constant(3, 4, 1);
  J(1, 0) = J(1, 3) = J(2, 1)= J(2, 2) = -1;
  J.row(2)/=(halfTrack+halfWheebase);
  omniSteeringMeasure.covariance = J*J.transpose();
}

}  // namespace romea
