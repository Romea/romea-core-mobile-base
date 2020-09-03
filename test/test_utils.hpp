#include "romea_odo/kinematic/KinematicMeasure.hpp"

void compareKinematicMeasure(const romea::KinematicMeasure & measure1,
                             const romea::KinematicMeasure & measure2)
{
//  std::cout <<" measure1 "<< std::endl;
//  std::cout << measure1 << std::endl;

//  std::cout <<" measure2 "<< std::endl;
//  std::cout << measure2 << std::endl;

  ASSERT_NEAR(measure1.longitudinalSpeed,measure2.longitudinalSpeed,0.001);
  ASSERT_NEAR(measure2.lateralSpeed,measure2.lateralSpeed,0.001);
  ASSERT_NEAR(measure2.instantaneousCurvature,measure2.instantaneousCurvature,0.001);
  ASSERT_NEAR(measure2.angularSpeed,measure2.angularSpeed,0.001);
  ASSERT_NEAR(0,(measure1.covariance.array() - measure2.covariance.array()).sum(),0.001);

}
