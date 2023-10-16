set(SOURCE_FILES
    main.cpp
)
set(HEADER_FILES
)
add_executable(epimetheus
    ${SOURCE_FILES} ${HEADER_FILES}
)
include(Install.cmake)
