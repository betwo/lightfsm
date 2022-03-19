if (CATKIN_ENABLE_TESTING)

catkin_add_gtest(${PROJECT_NAME}_tests
    action_test.cpp
    event_test.cpp
    meta_state_test.cpp
)
target_link_libraries(${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtest_main
)

endif()