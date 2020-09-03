cmake_minimum_required(VERSION 2.8.3)
project(romea_odo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS romea_common)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    )

###########
## Build ##
###########

include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
    )

add_library(romea_odo
    src/odometry/OdometryFrame.cpp
    src/odometry/OdometryFrame1FAS2RWD.cpp
    src/odometry/OdometryFrame1FAS2FWD.cpp
    src/odometry/OdometryFrame2AS4WD.cpp
    src/odometry/OdometryFrame2WD.cpp
    src/odometry/OdometryFrame4WD.cpp
    src/odometry/OdometryFrame1FWS2RWD.cpp
    src/odometry/OdometryFrame2FWS2RWD.cpp
    src/odometry/OdometryFrame2FWS2FWD.cpp
    src/odometry/OdometryFrame4WS4WD.cpp
    src/kinematic/axle_steering/OneAxleSteeringKinematic.cpp
    src/kinematic/axle_steering/OneAxleSteeringCommand.cpp
    src/kinematic/axle_steering/OneAxleSteeringMeasure.cpp
    src/kinematic/axle_steering/OneAxleSteeringConstraints.cpp
    src/kinematic/axle_steering/TwoAxleSteeringKinematic.cpp
    src/kinematic/axle_steering/TwoAxleSteeringCommand.cpp
    src/kinematic/axle_steering/TwoAxleSteeringConstraints.cpp
    src/kinematic/axle_steering/TwoAxleSteeringMeasure.cpp
    src/kinematic/axle_steering/FowardOneAxleSteeringKinematic.cpp
    src/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.cpp
    src/kinematic/axle_steering/InverseOneAxleSteeringKinematic.cpp
    src/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.cpp
    src/kinematic/skid_steering/SkidSteeringKinematic.cpp
    src/kinematic/skid_steering/SkidSteeringCommand.cpp
    src/kinematic/skid_steering/SkidSteeringMeasure.cpp
    src/kinematic/skid_steering/SkidSteeringConstraints.cpp
    src/kinematic/skid_steering/ForwardSkidSteeringKinematic.cpp
    src/kinematic/skid_steering/InverseSkidSteeringKinematic.cpp
    src/kinematic/omni_steering/OmniSteeringCommand.cpp
    src/kinematic/omni_steering/OmniSteeringMeasure.cpp
    src/kinematic/omni_steering/OmniSteeringConstraints.cpp
    src/kinematic/omni_steering/MecanumWheelSteeringKinematic.cpp
    src/kinematic/omni_steering/ForwardMecanumWheelSteeringKinematic.cpp
    src/kinematic/omni_steering/InverseMecanumWheelSteeringKinematic.cpp
    src/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.cpp
    src/kinematic/wheel_steering/FowardFourWheelSteeringKinematic.cpp
    src/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.cpp
    src/kinematic/wheel_steering/InverseFourWheelSteeringKinematic.cpp
    src/kinematic/wheel_steering/FourWheelSteeringKinematic.cpp
    src/kinematic/wheel_steering/TwoWheelSteeringKinematic.cpp
    src/kinematic/KinematicMeasure.cpp
    )

target_link_libraries(romea_odo ${catkin_LIBRARIES} yaml-cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trackbod_driver trackbod_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif(CATKIN_ENABLE_TESTING)
