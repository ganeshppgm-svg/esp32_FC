cmake_minimum_required(VERSION 3.16)
project(flight_core LANGUAGES CXX)

add_library(flight_core STATIC
  src/pid.cpp
  src/attitude_estimator.cpp
  src/fc_controller.cpp
)
target_include_directories(flight_core PUBLIC include)
target_compile_features(flight_core PUBLIC cxx_std_17)
target_compile_options(flight_core PRIVATE -Wall -Wextra -Wpedantic)
