find_package(glfw3 REQUIRED FATAL_ERROR) # error
find_package(Vulkan REQUIRED FATAL_ERROR) # error
find_package(VulkanHeaders CONFIG)
include(FindGLM.cmake)

target_include_directories (epimetheus
    PUBLIC ${GLFW_INCLUDE_DIRS}
    PUBLIC ${Vulkan_INCLUDE_DIRS}
)
target_link_libraries (epimetheus glfw)
target_link_libraries (epimetheus ${VULKAN_LIB_LIST})
