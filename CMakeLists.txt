cmake_minimum_required(VERSION 3.1.0)
project(sbc15_fsm)

add_definitions(-W -Wall -Wextra)

## Enforce that we use C++17
set(CMAKE_CXX_STANDARD 17)


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

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)


find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
)

add_library(lightfsm
  src/lightfsm/action.cpp
  src/lightfsm/actions.cpp
  src/lightfsm/event.cpp
  src/lightfsm/guard.cpp
  src/lightfsm/meta_state.cpp
  src/lightfsm/state_machine_basic_executor.cpp
  src/lightfsm/state_machine_executor.cpp
  src/lightfsm/state_machine.cpp
  src/lightfsm/state.cpp
  src/lightfsm/transition.cpp
  src/lightfsm/triggered_event.cpp
)
target_include_directories(lightfsm
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_library(lightfsm_ros
  src/lightfsm_ros/state_machine_ros_executor.cpp
)
target_link_libraries(lightfsm_ros
  lightfsm
)

add_library(sbc15_state_machine
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
  src/states/place_cup.cpp
  src/states/store_object.cpp
  src/states/place_object.cpp

  src/states/visual_servoing.cpp
  src/states/gripper_state.cpp
  src/states/preplanned_state.cpp
  src/states/recorded_trajectory.cpp
  src/states/rest_to_pre_position.cpp
  src/states/moveit_motion.cpp

  src/states/position_to_base.cpp

  src/states/approach_object.cpp
  src/states/go_to_base.cpp
  src/states/deliver_object.cpp

  src/utils/map_explorer.cpp

  src/global.cpp
)
add_dependencies(sbc15_state_machine
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sbc15_state_machine
  lightfsm_ros
  ${OpenCV_LIBS} ${catkin_LIBRARIES}
)
add_dependencies(sbc15_state_machine ${catkin_EXPORTED_TARGETS})

add_executable(sbc15_state_machine_node
    src/state_machine_node.cpp
)
target_link_libraries(sbc15_state_machine_node
  sbc15_state_machine ${catkin_LIBRARIES}
)

add_subdirectory(src/examples)
add_subdirectory(src/tests)


# RVIZ
set(SOURCE_FILES
  src/rviz/exploration_control.cpp
)

add_library(${PROJECT_NAME}_rviz ${SOURCE_FILES})
set_property(TARGET ${PROJECT_NAME}_rviz PROPERTY AUTOMOC ON)

target_link_libraries(${PROJECT_NAME}_rviz Qt5::Core Qt5::Widgets ${catkin_LIBRARIES})


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

