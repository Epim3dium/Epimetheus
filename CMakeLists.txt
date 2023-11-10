cmake_minimum_required(VERSION 3.12)

project(epimetheus
  VERSION 1.0
  LANGUAGES CXX
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS_DEBUG "-O3")
set(CMAKE_C_FLAGS_DEBUG "-O3")

add_subdirectory(dependancies)
add_subdirectory(src)

