enable_testing()
set(SOURCE_FILES
    types.cpp
    cell.cpp
    grid.cpp

    io/app.cpp

    memory/pool_allocator.cpp

    debug/log.cpp

    math/math_func.cpp
    math/geometry_func.cpp


    physics/col_utils.cpp
    physics/physics_manager.cpp
    physics/restraint.cpp
    physics/rigidbody.cpp
    physics/solver.cpp
)
set(HEADER_FILES
    types.hpp
    cell.hpp
    grid.hpp

    multithreading/thread_pool.hpp

    memory/pool_allocator.hpp
    memory/allocator.hpp

    math/math_func.hpp
    math/math_defs.hpp
    math/math.hpp
    math/geometry_func.hpp


    io/app.hpp
    io/event_handler.hpp

    templates/buffer.hpp
    templates/component_group.hpp

    templates/set.hpp
    templates/stack_linked_list.hpp

    debug/log.hpp
    debug/debug.hpp

    physics/col_utils.hpp
    physics/physics_manager.hpp
    physics/restraint.hpp
    physics/rigidbody.hpp
    physics/solver.hpp
    physics/collider.hpp
    physics/material.hpp
    physics/transform.hpp
)
set(TEST_FILES
    main.cpp
    # tests/group_test.cpp
    # tests/component_group_test.cpp
    # tests/entity_test.cpp
)

add_executable(epimetheus
    ${TEST_FILES} 
    ${SOURCE_FILES} ${HEADER_FILES}
)
include(GoogleTest)
# gtest_discover_tests(epimetheus)


include_directories(.)
include(../cmake/Install.cmake)

add_compile_options(
    "$<$<CONFIG:RELEASE>:-o3>"
    "$<$<CONFIG:DEBUG>:-glldb>"
)
