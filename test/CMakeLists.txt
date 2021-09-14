find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_1AS2FWD test_1AS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_1AS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_1AS2FWD PRIVATE -std=c++17)
add_test(test_1as ${PROJECT_NAME}_test_1AS2FWD)

add_executable(${PROJECT_NAME}_test_1AS2RWD test_1AS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_1AS2RWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_1AS2RWD PRIVATE -std=c++17)
add_test(test_1as ${PROJECT_NAME}_test_1AS2RWD)

add_executable(${PROJECT_NAME}_test_2AS4WD test_2AS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_2AS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2AS4WD PRIVATE -std=c++17)
add_test(test_2as ${PROJECT_NAME}_test_2AS4WD)

add_executable(${PROJECT_NAME}_test_2FWS24WD test_2FWS2RWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_2FWS24WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2FWS24WD PRIVATE -std=c++17)
add_test(test_2ws ${PROJECT_NAME}_test_2FWS24WD)

add_executable(${PROJECT_NAME}_test_2FWS4WD test_2FWS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_2FWS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2FWS4WD PRIVATE -std=c++17)
add_test(test_2ws ${PROJECT_NAME}_test_2FWS4WD)

add_executable(${PROJECT_NAME}_test_2FWS2FWD test_2FWS2FWD.cpp )
target_link_libraries(${PROJECT_NAME}_test_2FWS2FWD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2FWS2FWD PRIVATE -std=c++17)
add_test(test_2ws ${PROJECT_NAME}_test_2FWS2FWD)

add_executable(${PROJECT_NAME}_test_4WS4WD test_4WS4WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_4WS4WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_4WS4WD PRIVATE -std=c++17)
add_test(test_4ws ${PROJECT_NAME}_test_4WS4WD)

add_executable(${PROJECT_NAME}_test_2WD test_2WD.cpp )
target_link_libraries(${PROJECT_NAME}_test_2WD ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2WD PRIVATE -std=c++17)
add_test(test_ss ${PROJECT_NAME}_test_2WD)


