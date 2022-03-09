 cmake_minimum_required(VERSION 3.5)

project(
  "romea_core_mobile_base"
  VERSION 0.1.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(romea_core_common REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/odometry/OdometryFrame1FAS2RWD.cpp
  src/odometry/OdometryFrame1FAS2FWD.cpp
  src/odometry/OdometryFrame2AS4WD.cpp
  src/odometry/OdometryFrame2WD.cpp
  src/odometry/OdometryFrame4WD.cpp
  src/odometry/OdometryFrame1FWS2RWD.cpp
  src/odometry/OdometryFrame2FWS2RWD.cpp
  src/odometry/OdometryFrame2FWS2FWD.cpp
  src/odometry/OdometryFrame2FWS4WD.cpp
  src/odometry/OdometryFrame4WS4WD.cpp
  src/kinematic/axle_steering/OneAxleSteeringKinematic.cpp
  src/kinematic/axle_steering/OneAxleSteeringCommand.cpp
  src/kinematic/axle_steering/OneAxleSteeringCommandLimits.cpp
  src/kinematic/axle_steering/OneAxleSteeringMeasure.cpp
  src/kinematic/axle_steering/TwoAxleSteeringKinematic.cpp
  src/kinematic/axle_steering/TwoAxleSteeringCommand.cpp
  src/kinematic/axle_steering/TwoAxleSteeringCommandLimits.cpp
  src/kinematic/axle_steering/TwoAxleSteeringMeasure.cpp
  src/kinematic/axle_steering/FowardOneAxleSteeringKinematic.cpp
  src/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.cpp
  src/kinematic/axle_steering/InverseOneAxleSteeringKinematic.cpp
  src/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.cpp
  src/kinematic/skid_steering/SkidSteeringKinematic.cpp
  src/kinematic/skid_steering/SkidSteeringCommand.cpp
  src/kinematic/skid_steering/SkidSteeringMeasure.cpp
  src/kinematic/skid_steering/SkidSteeringCommandLimits.cpp
  src/kinematic/skid_steering/ForwardSkidSteeringKinematic.cpp
  src/kinematic/skid_steering/InverseSkidSteeringKinematic.cpp
  src/kinematic/omni_steering/OmniSteeringCommand.cpp
  src/kinematic/omni_steering/OmniSteeringMeasure.cpp
  src/kinematic/omni_steering/OmniSteeringCommandLimits.cpp
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
  src/kinematic/CommandLimits.cpp
  src/info/MobileBaseControl.cpp
  src/info/MobileBaseGeometry.cpp
  src/info/MobileBaseInertia.cpp
  src/info/MobileBaseInfo1FAS2RWD.cpp
  src/info/MobileBaseInfo1FAS2FWD.cpp
  src/info/MobileBaseInfo1FWS2RWD.cpp
  src/info/MobileBaseInfo2AS4WD.cpp
  src/info/MobileBaseInfo2FWS2FWD.cpp
  src/info/MobileBaseInfo2FWS2RWD.cpp
  src/info/MobileBaseInfo2FWS4WD.cpp
  src/info/MobileBaseInfo2TD.cpp
  src/info/MobileBaseInfo2THD.cpp
  src/info/MobileBaseInfo2WD.cpp
  src/info/MobileBaseInfo4WD.cpp
  src/info/MobileBaseInfo4WS4WD.cpp

  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${romea_core_common_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3 -std=c++17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_common::romea_core_common)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

option(BUILD_TESTS "BUILD WITH TESTS" OFF)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif(BUILD_TESTS)
