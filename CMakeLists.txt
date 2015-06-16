cmake_minimum_required(VERSION 2.8.3)
project(sbc15_fsm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sbc15_msgs
  smach
  roscpp 
  std_msgs 
  tf
  geometry_msgs
  visualization_msgs
  path_msgs
)
add_definitions(-W -Wall)

find_package(catkin REQUIRED )

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

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
add_dependencies(sbc15_state_machine sick_msgs_generate_messages_cpp path_msgs_generate_messages_cpp)
target_link_libraries(sbc15_state_machine ${OpenCV_LIBS})

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
