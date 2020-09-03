catkin_add_gtest(${PROJECT_NAME}_test_1as test_1as.cpp )
target_link_libraries(${PROJECT_NAME}_test_1as ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_2as test_2as.cpp )
target_link_libraries(${PROJECT_NAME}_test_2as ${PROJECT_NAME} ${catkin_LIBRARIES})

#catkin_add_gtest(${PROJECT_NAME}_test_2ws test/test_2ws.cpp )
#target_link_libraries(${PROJECT_NAME}_test_2ws ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_4ws test_4ws.cpp )
target_link_libraries(${PROJECT_NAME}_test_4ws ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_ss test_ss.cpp )
target_link_libraries(${PROJECT_NAME}_test_ss ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_frame test_frame.cpp )
target_link_libraries(${PROJECT_NAME}_test_frame ${PROJECT_NAME} ${catkin_LIBRARIES})


