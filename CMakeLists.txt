cmake_minimum_required(VERSION 3.16)
project(VirtualBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(virtual_bridge_core
  src/AppConfig.cpp
  src/ControlFrame.cpp
  src/RobotPositionJson.cpp
  src/TerminalStatus.cpp
  src/VehicleModel.cpp
)

target_include_directories(virtual_bridge_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(virtual_aruco_bridge
  src/virtual_aruco_bridge.cpp
)
target_link_libraries(virtual_aruco_bridge PRIVATE virtual_bridge_core)

enable_testing()

add_executable(virtual_bridge_tests
  tests/virtual_bridge_tests.cpp
)
target_link_libraries(virtual_bridge_tests PRIVATE virtual_bridge_core)
add_test(NAME virtual_bridge_tests COMMAND virtual_bridge_tests)
set_tests_properties(virtual_bridge_tests PROPERTIES
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
