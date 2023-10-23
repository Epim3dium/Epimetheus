find_package(GLM REQUIRED)
find_package(SFML 2.6 COMPONENTS graphics audio REQUIRED)

target_include_directories (epimetheus
    PUBLIC ${GLM_INCLUDE_DIR}
    PUBLIC ${SFML_INCLUDE_DIR}
)
target_link_libraries (epimetheus 
    ${GLM_LIB_LIST}
    sfml-graphics sfml-audio
)
