cmake_minimum_required(VERSION 3.10.0)
project(lightfsm)

add_definitions(-W -Wall -Wextra)

## Enforce that we use C++17
set(CMAKE_CXX_STANDARD 17)
set(CONFIG_NAMESPACE_CMAKE "betwo")
set(PACKAGE_VERSION 0.0.0)  # Use git tags ??

enable_testing()

add_library(${PROJECT_NAME}
  SHARED
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
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_subdirectory(examples)
add_subdirectory(tests)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
generate_export_header(lightfsm)

write_basic_package_version_file(
  lightfsmVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)

set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})


write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/lightfsm-config-version.cmake"
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${PROJECT_NAME}-config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
)

install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${CONFIG_NAMESPACE_CMAKE}::
  DESTINATION ${ConfigPackageLocation}
)

install(
  FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION ${ConfigPackageLocation}
  COMPONENT Development
)
