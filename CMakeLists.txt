cmake_minimum_required(VERSION 3.5)
project(ros2_qualisys_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(Sophus REQUIRED)
find_package(nav_msgs)
find_package(pendulum_utils REQUIRED)


set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    realtime_tools
    eigen3_cmake_module
    Eigen3
    Sophus
    nav_msgs
    pendulum_utils
)

set(QUALISYS_DRIVER_LIB qualisys_driver)
add_library(${QUALISYS_DRIVER_LIB} SHARED
            src/qualisys_driver_node.cpp
            include/ros2_qualisys_driver/RTProtocol.cpp
            include/ros2_qualisys_driver/Markup.cpp
            include/ros2_qualisys_driver/RTPacket.cpp
            include/ros2_qualisys_driver/Network.cpp
)

target_include_directories(${QUALISYS_DRIVER_LIB}
                            PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${QUALISYS_DRIVER_LIB} ${dependencies})

rclcpp_components_register_nodes(${QUALISYS_DRIVER_LIB} "QualisysDriver")

set(QUALISYS_DRIVER_EXE "${QUALISYS_DRIVER_LIB}_exe")
add_executable(${QUALISYS_DRIVER_EXE} src/qualisys_driver_node_main.cpp)
target_link_libraries(${QUALISYS_DRIVER_EXE} ${QUALISYS_DRIVER_LIB})

ament_export_targets(export_${QUALISYS_DRIVER_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wthread-safety)   # for your whole package
  # target_compile_options(${QUALISYS_DRIVER_EXE} PUBLIC -Wthread-safety)  # for a single library or executable
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${QUALISYS_DRIVER_LIB} ${QUALISYS_DRIVER_EXE}
  EXPORT export_${QUALISYS_DRIVER_LIB}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()
