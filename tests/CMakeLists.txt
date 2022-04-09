
find_package(catkin)

file(GLOB_RECURSE TEST_SOURCES "*.cpp")

if (${catkin_FOUND})
    catkin_add_gtest(${PROJECT_NAME}_tests
        ${TEST_SOURCES}
    )
    target_link_libraries(${PROJECT_NAME}_tests
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        gtest_main
    )

else()
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(${PROJECT_NAME}_tests
      ${TEST_SOURCES}
    )
    target_link_libraries(
      ${PROJECT_NAME}_tests
      ${PROJECT_NAME}
      GTest::GTest
      gtest_main
    )

    gtest_discover_tests(${PROJECT_NAME}_tests)
endif()