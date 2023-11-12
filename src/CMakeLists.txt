enable_testing()
set(SOURCE_FILES
    io/app.cpp

    memory/pool_allocator.cpp

    debug/log.cpp

    core/entity.cpp
)
set(HEADER_FILES
    multithreading/thread_pool.hpp

    memory/pool_allocator.hpp
    memory/allocator.hpp

    io/app.hpp
    io/event_handler.hpp

    core/buffer.hpp
    core/group.hpp
    core/component_group.hpp
    core/entity.hpp

    utils/set.hpp
    utils/stack_linked_list.hpp

    debug/log.hpp
    debug/debug.hpp
)
set(TEST_FILES
    main.cpp
    tests/group_test.cpp
    tests/component_group_test.cpp
    tests/entity_test.cpp
)

add_executable(epimetheus
    ${TEST_FILES} 
    ${SOURCE_FILES} ${HEADER_FILES}
)
include(GoogleTest)
gtest_discover_tests(epimetheus)


include_directories(.)
include(../cmake/Install.cmake)
