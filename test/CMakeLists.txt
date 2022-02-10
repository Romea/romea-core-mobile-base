find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_kinematic_1AS2FWD kinematic/test_kinematic_1AS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_1AS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_1AS2FWD PRIVATE -std=c++17)
add_test(test_kinematic_1as ${PROJECT_NAME}_test_kinematic_1AS2FWD)

add_executable(${PROJECT_NAME}_test_kinematic_1AS2RWD kinematic/test_kinematic_1AS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_1AS2RWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_1AS2RWD PRIVATE -std=c++17)
add_test(test_kinematic_1as ${PROJECT_NAME}_test_kinematic_1AS2RWD)

add_executable(${PROJECT_NAME}_test_kinematic_2AS4WD kinematic/test_kinematic_2AS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2AS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2AS4WD PRIVATE -std=c++17)
add_test(test_kinematic_2as ${PROJECT_NAME}_test_kinematic_2AS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2FWS24WD kinematic/test_kinematic_2FWS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2FWS24WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2FWS24WD PRIVATE -std=c++17)
add_test(test_kinematic_2ws ${PROJECT_NAME}_test_kinematic_2FWS24WD)

add_executable(${PROJECT_NAME}_test_kinematic_2FWS4WD kinematic/test_kinematic_2FWS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2FWS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2FWS4WD PRIVATE -std=c++17)
add_test(test_kinematic_2ws ${PROJECT_NAME}_test_kinematic_2FWS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2FWS2FWD kinematic/test_kinematic_2FWS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2FWS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2FWS2FWD PRIVATE -std=c++17)
add_test(test_kinematic_2ws ${PROJECT_NAME}_test_kinematic_2FWS2FWD)

add_executable(${PROJECT_NAME}_test_kinematic_4WS4WD kinematic/test_kinematic_4WS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_4WS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_4WS4WD PRIVATE -std=c++17)
add_test(test_kinematic_4ws ${PROJECT_NAME}_test_kinematic_4WS4WD)

add_executable(${PROJECT_NAME}_test_kinematic_2WD kinematic/test_kinematic_2WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_kinematic_2WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_kinematic_2WD PRIVATE -std=c++17)
add_test(test_kinematic_ss ${PROJECT_NAME}_test_kinematic_2WD)

add_executable(${PROJECT_NAME}_test_command_limits kinematic/test_command_limits.cpp )
target_link_libraries(${PROJECT_NAME}_test_command_limits ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_command_limits PRIVATE -std=c++17)
add_test(test_kinematic_ss ${PROJECT_NAME}_test_command_limits)

