cmake_minimum_required(VERSION 3.16)
project(fc_sil_ros2 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The ROS package lives at <repo>/sil_ws/src/fc_sil_ros2.
# Compile the exact same source files that PlatformIO discovers under lib/flight_core.
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(FC_CORE_DIR "${REPO_ROOT}/lib/flight_core")

add_library(flight_core STATIC
  "${FC_CORE_DIR}/src/pid.cpp"
  "${FC_CORE_DIR}/src/attitude_estimator.cpp"
  "${FC_CORE_DIR}/src/fc_controller.cpp"
)
target_include_directories(flight_core PUBLIC "${FC_CORE_DIR}/include")
target_compile_features(flight_core PUBLIC cxx_std_17)
target_compile_options(flight_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(fc_sil_node src/fc_sil_node.cpp)
target_link_libraries(fc_sil_node flight_core)
ament_target_dependencies(fc_sil_node rclcpp sensor_msgs actuator_msgs std_msgs)
target_compile_options(fc_sil_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS fc_sil_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

ament_package()
