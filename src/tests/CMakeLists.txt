if (CATKIN_ENABLE_TESTING)

catkin_add_gtest(sbc15_fsm_tests
    action_test.cpp
    event_test.cpp
    meta_state_test.cpp
)
target_link_libraries(sbc15_fsm_tests
    sbc15_state_machine
    ${catkin_LIBRARIES}
    gtest_main
)

endif()