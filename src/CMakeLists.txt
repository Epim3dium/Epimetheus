set(SOURCE_FILES
    main.cpp
    memory/pool_allocator.cpp
    debug/log.cpp
)
set(HEADER_FILES
    memory/pool_allocator.hpp
    memory/allocator.hpp
    utils/set.hpp
    utils/stack_linked_list.hpp
)
add_executable(epimetheus
    ${SOURCE_FILES} ${HEADER_FILES}
)
include_directories(.)
