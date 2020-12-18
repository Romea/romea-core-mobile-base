find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_1as test_1as.cpp )
target_link_libraries(${PROJECT_NAME}_test_1as ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_1as PRIVATE -std=c++17)
add_test(test_1as ${PROJECT_NAME}_test_1as)

add_executable(${PROJECT_NAME}_test_2as test_2as.cpp )
target_link_libraries(${PROJECT_NAME}_test_2as ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2as PRIVATE -std=c++17)
add_test(test_2as ${PROJECT_NAME}_test_2as)

add_executable(${PROJECT_NAME}_test_2ws test_2ws.cpp )
target_link_libraries(${PROJECT_NAME}_test_2ws ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_2ws PRIVATE -std=c++17)
add_test(test_2ws ${PROJECT_NAME}_test_2ws)

add_executable(${PROJECT_NAME}_test_4ws test_4ws.cpp )
target_link_libraries(${PROJECT_NAME}_test_4ws ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_4ws PRIVATE -std=c++17)
add_test(test_4ws ${PROJECT_NAME}_test_4ws)

add_executable(${PROJECT_NAME}_test_ss test_ss.cpp )
target_link_libraries(${PROJECT_NAME}_test_ss ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_ss PRIVATE -std=c++17)
add_test(test_ss ${PROJECT_NAME}_test_ss)

add_executable(${PROJECT_NAME}_test_frame test_frame.cpp )
target_link_libraries(${PROJECT_NAME}_test_frame ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_frame PRIVATE -std=c++17)
add_test(test_frame ${PROJECT_NAME}_test_frame)


