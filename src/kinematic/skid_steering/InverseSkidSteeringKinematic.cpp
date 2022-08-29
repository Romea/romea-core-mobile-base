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
void inverseKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2TD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  inverseKinematicImpl(parameters.wheelTrack,
                       odometryFrame.leftTrackLinearSpeed,
                       odometryFrame.rightTrackLinearSpeed,
                       parameters.wheelLinearSpeedVariance,
                       skidSteeringMeasure);

}

//-----------------------------------------------------------------------------
void inverseKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const OdometryFrame2WD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  inverseKinematicImpl(parameters.wheelTrack,
                       odometryFrame.leftWheelLinearSpeed,
                       odometryFrame.rightWheelLinearSpeed,
                       parameters.wheelLinearSpeedVariance,
                       skidSteeringMeasure);

}


//-----------------------------------------------------------------------------
void inverseKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const OdometryFrame4WD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  double leftWheelSpeed = SkidSteeringKinematic::
      minWheelLinearSpeed(odometryFrame.frontLeftWheelLinearSpeed,
                          odometryFrame.rearLeftWheelLinearSpeed);

  double rightWheelSpeed =SkidSteeringKinematic::
      minWheelLinearSpeed(odometryFrame.frontRightWheelLinearSpeed,
                          odometryFrame.rearRightWheelLinearSpeed);


  inverseKinematicImpl(parameters.wheelTrack,
                       leftWheelSpeed,
                       rightWheelSpeed,
                       parameters.wheelLinearSpeedVariance,
                       skidSteeringMeasure);
}

}//end romea
