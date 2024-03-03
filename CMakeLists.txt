cmake_minimum_required(VERSION 3.12)

project(epimetheus
  VERSION 1.0
  LANGUAGES CXX
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
add_compile_options(
    -glldb
)

if(POLICY CMP0079 )
  cmake_policy(SET CMP0079 NEW)
endif()

add_subdirectory(dependancies)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(demo)

include(cmake/Install.cmake)
