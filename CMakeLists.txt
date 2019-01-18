cmake_minimum_required(VERSION 3.9.6)
cmake_policy(SET CMP0048 NEW)

project(runner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CONFIG)
  set(CONFIG Release)
endif()

message(STATUS "Build in ${CONFIG} mode")

if(APPLE)
  set(CMAKE_MACOSX_RPATH 1)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

add_subdirectory(bin)
