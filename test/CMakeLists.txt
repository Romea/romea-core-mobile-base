find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_kinematic_1AS2FWD kinematic/test_kinematic_1AS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_1AS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_1AS2FWD PRIVATE -std=c++17)
add_test(test_kinematic_1AS2FWD ${PROJECT_NAME}_test_kinematic_1AS2FWD)

add_executable(${PROJECT_NAME}_test_kinematic_1AS2RWD kinematic/test_kinematic_1AS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_1AS2RWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_1AS2RWD PRIVATE -std=c++17)
add_test(test_kinematic_1AS2RWD ${PROJECT_NAME}_test_kinematic_1AS2RWD)

add_executable(${PROJECT_NAME}_test_kinematic_1AS4WD kinematic/test_kinematic_1AS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_1AS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_1AS4WD PRIVATE -std=c++17)
add_test(test_kinematic_1AS4RWD ${PROJECT_NAME}_test_kinematic_1AS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2AS4WD kinematic/test_kinematic_2AS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2AS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2AS4WD PRIVATE -std=c++17)
add_test(test_kinematic_2AS4WD ${PROJECT_NAME}_test_kinematic_2AS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2FWS2RWD kinematic/test_kinematic_2FWS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2FWS2RWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2FWS2RWD PRIVATE -std=c++17)
add_test(test_kinematic_2FWS2RWD ${PROJECT_NAME}_test_kinematic_2FWS2RWD)

add_executable(${PROJECT_NAME}_test_kinematic_2FWS4WD kinematic/test_kinematic_2FWS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2FWS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2FWS4WD PRIVATE -std=c++17)
add_test(test_kinematic_2FWS4WD ${PROJECT_NAME}_test_kinematic_2FWS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2FWS2FWD kinematic/test_kinematic_2FWS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2FWS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2FWS2FWD PRIVATE -std=c++17)
add_test(test_kinematic_2FWS2FWD ${PROJECT_NAME}_test_kinematic_2FWS2FWD)

add_executable(${PROJECT_NAME}_test_kinematic_4WS4WD kinematic/test_kinematic_4WS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_4WS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_4WS4WD PRIVATE -std=c++17)
add_test(test_kinematic_4WS ${PROJECT_NAME}_test_kinematic_4WS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2WD kinematic/test_kinematic_2WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2WD PRIVATE -std=c++17)
add_test(test_kinematic_2WD ${PROJECT_NAME}_test_kinematic_2WD)

add_executable(${PROJECT_NAME}_test_command_limits kinematic/test_command_limits.cpp )
target_link_libraries(${PROJECT_NAME}_test_command_limits ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_command_limits PRIVATE -std=c++17)
add_test(test_command_limits ${PROJECT_NAME}_test_command_limits)

add_executable(${PROJECT_NAME}_test_simulation_1FAS2FWD simulation/test_simulation_1FAS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_1FAS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_1FAS2FWD PRIVATE -std=c++17)
add_test(test_simulation_control_1FAS2FWD ${PROJECT_NAME}_test_simulation_1FAS2FWD)

add_executable(${PROJECT_NAME}_test_simulation_1FAS2RWD simulation/test_simulation_1FAS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_1FAS2RWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_1FAS2RWD PRIVATE -std=c++17)
add_test(test_simulation_control_1FAS2RWD ${PROJECT_NAME}_test_simulation_1AS2RWD)

add_executable(${PROJECT_NAME}_test_simulation_1FAS4WD simulation/test_simulation_1FAS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_1FAS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_1FAS4WD PRIVATE -std=c++17)
add_test(test_simulation_control_1FAS4WD ${PROJECT_NAME}_test_simulation_1AS4WD)

add_executable(${PROJECT_NAME}_test_simulation_2AS4WD simulation/test_simulation_2AS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_2AS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_2AS4WD PRIVATE -std=c++17)
add_test(test_simulation_control_2AS4WD ${PROJECT_NAME}_test_simulation_2AS4WD)

add_executable(${PROJECT_NAME}_test_simulation_2FWS2FWD simulation/test_simulation_2FWS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_2FWS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_2FWS2FWD PRIVATE -std=c++17)
add_test(test_simulation_control_2FWS2FWD ${PROJECT_NAME}_test_simulation_1AS2RWD)

add_executable(${PROJECT_NAME}_test_simulation_2FWS2RWD simulation/test_simulation_2FWS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_2FWS2RWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_2FWS2RWD PRIVATE -std=c++17)
add_test(test_simulation_control_2FWS2RWD ${PROJECT_NAME}_test_simulation_1AS2RWD)

add_executable(${PROJECT_NAME}_test_simulation_2TD simulation/test_simulation_2TD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_2TD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_2TD PRIVATE -std=c++17)
add_test(test_simulation_control_2TD ${PROJECT_NAME}_test_simulation_2TD)

add_executable(${PROJECT_NAME}_test_simulation_2THD simulation/test_simulation_2THD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_2THD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_2THD PRIVATE -std=c++17)
add_test(test_simulation_control_2THD ${PROJECT_NAME}_test_simulation_2THD)

add_executable(${PROJECT_NAME}_test_simulation_2TTD simulation/test_simulation_2TTD.cpp )
target_link_libraries(${PROJECT_NAME}_test_simulation_2TTD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simulation_2TTD PRIVATE -std=c++17)
add_test(test_simulation_control_2TTD ${PROJECT_NAME}_test_simulation_2TTD)

add_executable(${PROJECT_NAME}_test_mobile_base_type info/test_mobile_base_type.cpp )
target_link_libraries(${PROJECT_NAME}_test_mobile_base_type ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_mobile_base_type PRIVATE -std=c++17)
add_test(test_mobile_base_type ${PROJECT_NAME}_test_mobile_base_type)
