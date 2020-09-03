#include "romea_odo/kinematic/omni_steering/InverseMecanumWheelSteeringKinematic.hpp"
#include <iostream>

namespace romea
{

void inverseKinematic(const MecanumWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame4WD & odometryFrame,
                      OmniSteeringMeasure & omniSteeringMeasure)
{
  const double halfTrack = parameters.track/2.;
  const double halfWheebase = parameters.wheelbase/2;

  omniSteeringMeasure.longitudinalSpeed=
      MecanumWheelSteeringKinematic::computeLongitudinalSpeed(odometryFrame.frontLeftWheelSpeed,
                                                              odometryFrame.frontRightWheelSpeed,
                                                              odometryFrame.rearLeftWheelSpeed,
                                                              odometryFrame.rearRightWheelSpeed);

  omniSteeringMeasure.lateralSpeed =
      MecanumWheelSteeringKinematic::computeLateralSpeed(odometryFrame.frontRightWheelSpeed,
                                                         odometryFrame.rearLeftWheelSpeed,
                                                         odometryFrame.frontLeftWheelSpeed,
                                                         odometryFrame.rearRightWheelSpeed);

  omniSteeringMeasure.angularSpeed =
      MecanumWheelSteeringKinematic::computeAngularSpeed(odometryFrame.frontRightWheelSpeed,
                                                         odometryFrame.rearLeftWheelSpeed,
                                                         odometryFrame.frontLeftWheelSpeed,
                                                         odometryFrame.rearRightWheelSpeed,
                                                         halfWheebase,
                                                         halfTrack);


  Eigen::MatrixXd J = Eigen::MatrixXd::Constant(3,4,1);
  J(1,0)=J(1,3)=J(2,1)=J(2,2)=-1;
  J.row(2)/=(halfTrack+halfWheebase);
  omniSteeringMeasure.covariance=J*J.transpose();

}



}//end romea
