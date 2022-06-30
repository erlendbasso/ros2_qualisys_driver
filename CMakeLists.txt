cmake_minimum_required(VERSION 3.10)
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
#include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs)
find_package(geometry_msgs)
# find_package(pendulum_control REQUIRED)


set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    eigen3_cmake_module
    Eigen3
    nav_msgs
    # pendulum_control
    geometry_msgs
)

set(QUALISYS_DRIVER_LIB qualisys_driver)
add_library(${QUALISYS_DRIVER_LIB} 
            src/qualisys_driver_node.cpp
            include/ros2_qualisys_driver/RTProtocol.cpp
            include/ros2_qualisys_driver/Markup.cpp
            include/ros2_qualisys_driver/RTPacket.cpp
            include/ros2_qualisys_driver/Network.cpp
)

target_include_directories(${QUALISYS_DRIVER_LIB}
                            PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                            $<INSTALL_INTERFACE:include>
                            ${rclcpp_components_INCLUDE_DIRS}
                            ${lifecycle_msgs_INCLUDE_DIRS}
                            ${nav_msgs_INCLUDE_DIRS}
                            ${geometry_msgs_INCLUDE_DIRS}
                            ${rclcpp_lifecycle_INCLUDE_DIRS}
)

# ament_target_dependencies(${QUALISYS_DRIVER_LIB} ${dependencies})
target_link_libraries(${QUALISYS_DRIVER_LIB} PUBLIC 
  Eigen3::Eigen
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rclcpp_components_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  $<$<STREQUAL:"${CMAKE_CXX_COMPILER_ID}","MSVC">:ws2_32.lib>
)

#rclcpp_components_register_nodes(${QUALISYS_DRIVER_LIB} "QualisysDriver")

set(QUALISYS_DRIVER_EXE "${QUALISYS_DRIVER_LIB}_exe")
add_executable(${QUALISYS_DRIVER_EXE} src/qualisys_driver_node_main.cpp)
target_link_libraries(${QUALISYS_DRIVER_EXE} ${QUALISYS_DRIVER_LIB})

ament_export_targets(export_${QUALISYS_DRIVER_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wthread-safety)   # for your whole package
  # target_compile_options(${QUALISYS_DRIVER_EXE} PUBLIC -Wthread-safety)  # for a single library or executable
endif()

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

target_compile_features(${QUALISYS_DRIVER_EXE} PUBLIC cxx_std_14)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
