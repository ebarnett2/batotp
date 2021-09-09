cmake_minimum_required(VERSION 3.15)
include_guard(GLOBAL)

project(batotp)

option(BUILD_EXAMPLES "Build example programs" ON)

add_subdirectory(batotp)

if (BUILD_EXAMPLES)
  add_subdirectory(test)
endif()
