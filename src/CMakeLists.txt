enable_testing()
set(SOURCE_FILES
    core/io/app.cpp

    core/memory/pool_allocator.cpp

    debug/log.cpp

    core/entity.cpp
)
set(HEADER_FILES
    core/multithreading/thread_pool.hpp

    core/memory/pool_allocator.hpp
    core/memory/allocator.hpp

    core/io/app.hpp
    core/io/event_handler.hpp

    core/buffer.hpp
    core/group.hpp
    core/component_group.hpp
    core/entity.hpp

    core/utils/set.hpp
    core/utils/stack_linked_list.hpp

    debug/log.hpp
    debug/debug.hpp
)
set(TEST_FILES
    main.cpp
    tests/group_test.cpp
    tests/component_group_test.cpp
)

add_executable(epimetheus
    ${TEST_FILES} 
    ${SOURCE_FILES} ${HEADER_FILES}
)
include(GoogleTest)
gtest_discover_tests(epimetheus)


include_directories(.)
