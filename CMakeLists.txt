cmake_minimum_required(VERSION 3.12)

project(epimetheus
  VERSION 1.0
  LANGUAGES CXX
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)

if(POLICY CMP0079 )
  cmake_policy(SET CMP0079 NEW)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_C_FLAGS_DEBUG "-O0")

include(./cmake/Install.cmake)
include(./cmake/FindGLM.cmake)
