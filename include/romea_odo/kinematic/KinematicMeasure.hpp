#ifndef romea_KinematicMeasure_hpp
#define romea_KinematicMeasure_hpp

//eigen
#include <Eigen/Core>

namespace romea {


struct KinematicMeasure
{
  KinematicMeasure();
  double longitudinalSpeed;
  double lateralSpeed;
  double angularSpeed;
  double instantaneousCurvature;
  Eigen::Matrix4d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


std::ostream& operator<<(std::ostream& os, const KinematicMeasure & measure);

}//end romea
#endif
