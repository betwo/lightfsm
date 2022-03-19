cmake_minimum_required(VERSION 3.1.0)
project(lightfsm)

add_definitions(-W -Wall -Wextra)

## Enforce that we use C++17
set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED)


include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(lightfsm
  src/action.cpp
  src/actions.cpp
  src/event.cpp
  src/guard.cpp
  src/meta_state.cpp
  src/state_machine_basic_executor.cpp
  src/state_machine_executor.cpp
  src/state_machine.cpp
  src/state.cpp
  src/transition.cpp
  src/triggered_event.cpp
)
target_include_directories(lightfsm
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_subdirectory(examples)
add_subdirectory(tests)
