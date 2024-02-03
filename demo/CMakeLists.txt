add_executable(
    demo
    main.cpp
)
include_directories(../src)

target_link_libraries(demo
  PRIVATE
    epimetheus
)
