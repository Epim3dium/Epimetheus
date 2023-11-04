enable_testing()
set(SOURCE_FILES
    main.cpp
    memory/pool_allocator.cpp
    debug/log.cpp
    core/entity.cpp
)
set(HEADER_FILES
    memory/pool_allocator.hpp
    memory/allocator.hpp
    core/buffer.hpp
    core/group.hpp
    multithreading/thread_pool.hpp
    core/component_group.hpp
    core/entity.hpp
    debug/log.hpp
    debug/debug.hpp
    utils/set.hpp
    utils/stack_linked_list.hpp
)
set(TEST_FILES
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
