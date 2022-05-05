//romea
#include "romea_core_mobile_base/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace {

//--------------------------------------------------------------------------
void inverseKinematicImpl(const double & wheelTrack,
                          const double & leftWheelSpeed,
                          const double & rightWheelSpeed,
                          const double & wheelSpeedVariance,
                          romea::SkidSteeringMeasure & skidSteeringMeasure)
{
  skidSteeringMeasure.longitudinalSpeed = romea::SkidSteeringKinematic::computeLinearSpeed(leftWheelSpeed,rightWheelSpeed);
  skidSteeringMeasure.angularSpeed = romea::SkidSteeringKinematic::computeAngularSpeed(leftWheelSpeed,rightWheelSpeed,wheelTrack);
  skidSteeringMeasure.covariance << 0.5, 1/wheelTrack, 1/wheelTrack, 1/(wheelTrack*wheelTrack);
  skidSteeringMeasure.covariance*= wheelSpeedVariance;
}


}

namespace romea {



//-----------------------------------------------------------------------------
void inverseKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const OdometryFrame2WD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  inverseKinematicImpl(parameters.wheelTrack,
                       odometryFrame.leftWheelSpeed,
                       odometryFrame.rightWheelSpeed,
                       parameters.wheelSpeedVariance,
                       skidSteeringMeasure);

}


//-----------------------------------------------------------------------------
void inverseKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const OdometryFrame4WD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  double leftWheelSpeed = SkidSteeringKinematic::minWheelSpeed(odometryFrame.frontLeftWheelSpeed,
                                                               odometryFrame.rearLeftWheelSpeed);

  double rightWheelSpeed =SkidSteeringKinematic::minWheelSpeed(odometryFrame.frontRightWheelSpeed,
                                                               odometryFrame.rearRightWheelSpeed);


  inverseKinematicImpl(parameters.wheelTrack,
                       leftWheelSpeed,
                       rightWheelSpeed,
                       parameters.wheelSpeedVariance,
                       skidSteeringMeasure);
}

}//end romea
