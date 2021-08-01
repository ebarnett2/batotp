cmake_minimum_required(VERSION 3.15)
include_guard(GLOBAL)

project(batotp_test) 

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} batotp_lib)
