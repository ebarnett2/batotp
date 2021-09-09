cmake_minimum_required(VERSION 3.15)
include_guard(GLOBAL)

project(batotp_lib) 

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME} PRIVATE
  ba.cpp
  robot.cpp
  spline.cpp
  util.cpp
)

add_library(${PROJECT_NAME}_if INTERFACE)

target_sources(${PROJECT_NAME}_if INTERFACE
  ba.h
  config.h
  robot.h
  spline.h
  util.h
)

target_include_directories(${PROJECT_NAME}_if INTERFACE .)
target_link_libraries(${PROJECT_NAME}_if INTERFACE Eigen3::Eigen)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_if
)
