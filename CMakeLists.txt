cmake_minimum_required(VERSION 2.8.3)
project(sbc15_fsm)

add_definitions(-W -Wall)

set(CMAKE_BUILD_TYPE Debug)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  sbc15_msgs
  roscpp 
  std_msgs 
  tf
  geometry_msgs
  visualization_msgs
  path_msgs
  actionlib
  rviz
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)


find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
)

set(FSM
    src/fsm/state_machine.cpp
    src/fsm/state.cpp
    src/fsm/meta_state.cpp
    src/fsm/transition.cpp
    src/fsm/event.cpp
    src/fsm/triggered_event.cpp
    src/fsm/action.cpp
    src/fsm/guard.cpp
)
set(STATES
    src/states/global_state.cpp
    src/states/wait.cpp
    src/states/back_up.cpp
    src/states/explore_map.cpp
    src/states/wait_for_go_signal.cpp
    src/states/follow_path.cpp

    src/states/select_task.cpp
    src/states/explore.cpp
    src/states/fetch_object.cpp
    src/states/goto_object.cpp
    src/states/pickup_object.cpp
    src/states/visual_servoing.cpp

    src/states/gripper_state.cpp

    src/states/preplanned_state.cpp
    src/states/plan_arm_motion.cpp
    src/states/store_object.cpp
    src/states/place_object.cpp
    src/states/moveit_motion.cpp
)
set(UTILS
    src/utils/map_explorer.cpp

    src/global.cpp
)


add_library(sbc15_state_machine
    ${FSM}
    ${STATES}
    ${UTILS}
)
add_dependencies(sbc15_state_machine
    sick_msgs_generate_messages_cpp
    path_msgs_generate_messages_cpp
    sbc15_msgs_generate_messages_cpp
)
target_link_libraries(sbc15_state_machine ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(sbc15_state_machine_node
    src/state_machine_node.cpp
)
target_link_libraries(sbc15_state_machine_node
  sbc15_state_machine ${catkin_LIBRARIES}
)


add_executable(sbc15_test_map_exploration
    src/tests/map_exploration_test.cpp
)
target_link_libraries(sbc15_test_map_exploration
  sbc15_state_machine ${catkin_LIBRARIES}
)


add_executable(sbc15_test_find_cup
    src/tests/find_cup_test.cpp
)
target_link_libraries(sbc15_test_find_cup
  sbc15_state_machine ${catkin_LIBRARIES}
)

add_executable(sbc15_test_pick_up_cup
    src/tests/pick_up_cup_test.cpp
)
target_link_libraries(sbc15_test_pick_up_cup
  sbc15_state_machine ${catkin_LIBRARIES}
)





add_executable(sbc15_pick_up
    src/tests/pickup.cpp
)
target_link_libraries(sbc15_pick_up
  sbc15_state_machine ${catkin_LIBRARIES}
)


# RVIZ

qt4_wrap_cpp(MOC_FILES
  src/rviz/exploration_control.h
)

set(SOURCE_FILES
  src/rviz/exploration_control.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME}_rviz ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}_rviz ${QT_LIBRARIES} ${catkin_LIBRARIES})


## Install rules

install(TARGETS
  ${PROJECT_NAME}_rviz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

