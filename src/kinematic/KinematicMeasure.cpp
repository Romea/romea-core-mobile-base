//romea
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"

namespace romea {

//-----------------------------------------------------------------------------
KinematicMeasure::KinematicMeasure():
  longitudinalSpeed(0),
  lateralSpeed(0),
  angularSpeed(0),
  instantaneousCurvature(0),
  covariance(Eigen::Matrix4d::Zero())
{

}


//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const KinematicMeasure & measure)
{
  os<<" Kinematic measure  "<<std::endl;;
  os<<" measured longitudinal speed  " << measure.longitudinalSpeed << std::endl;
  os<<" measured lateral speed " << measure.lateralSpeed << std::endl;
  os<<" measured angular speed " << measure.angularSpeed << std::endl;
  os<<" measured instantaneous curvature " << measure.instantaneousCurvature << std::endl;
  os<<" measured covariance matrix " << std::endl;
  os<< measure.covariance;
  return os;
}


}//end romea

