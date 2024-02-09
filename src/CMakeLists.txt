set(SOURCE_FILES
    #main.cpp
    io/app.cpp
    debug/log.cpp
    math/types.cpp
    math/math_func.cpp
    math/geometry_func.cpp
    physics/collider.cpp
    physics/transform.cpp
    core/hierarchy.cpp
    # physics/physics_manager.cpp
    # physics/restraint.cpp
    # physics/rigidbody.cpp
    # physics/solver.cpp
)
set(HEADER_FILES
    core/entity.hpp
    core/hierarchy.hpp
    core/group.hpp
    core/slice.hpp
    memory/parallel_iterator.hpp
    memory/pool_allocator.hpp
    memory/allocator.hpp
    debug/log.hpp
    debug/debug.hpp
    multithreading/thread_pool.hpp
    math/types.hpp
    math/shapes/AABB.hpp
    math/shapes/circle.hpp
    math/shapes/concave_polygon.hpp
    math/shapes/convex_polygon.hpp
    math/shapes/ray.hpp
    math/math_func.hpp
    math/math_defs.hpp
    math/math.hpp
    math/geometry_func.hpp
    io/app.hpp
    io/event_handler.hpp
    templates/set.hpp
    templates/stack_linked_list.hpp
    debug/log.hpp
    debug/debug.hpp
    physics/physics_manager.hpp
    physics/restraint.hpp
    physics/rigidbody.hpp
    physics/solver.hpp
    physics/collider.hpp
    physics/material.hpp
    physics/transform.hpp
)
include_directories(.)
add_library(epimetheus
    ${SOURCE_FILES} ${HEADER_FILES}
)

add_compile_options(
    "$<$<CONFIG:RELEASE>:-o3>"
    "$<$<CONFIG:DEBUG>:-glldb>"
)
