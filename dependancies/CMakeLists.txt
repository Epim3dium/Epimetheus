 include(FetchContent)

 set(SFML_VERSION 2.6.x)
 set(IMGUI_VERSION v1.89)
 set(IMGUI_SFML_VERSION 2.6.x)

 FetchContent_Declare(
     sfml
     GIT_REPOSITORY "https://github.com/SFML/SFML.git"
     GIT_TAG        "${SFML_VERSION}"
 )

 FetchContent_Declare(
     imgui
     GIT_REPOSITORY "https://github.com/ocornut/imgui.git"
     GIT_TAG        "${IMGUI_VERSION}"
 )

 FetchContent_Declare(
     imgui-sfml
     GIT_REPOSITORY "https://github.com/SFML/imgui-sfml.git"
     #GIT_TAG        "${IMGUI_SFML_VERSION}"
     # GIT_COMMIT 5f54b69b6aee18db846c81633f86f78c2586dded
     # ^ or like this - sometimes it's better because FetchContent won't look
     # into remote to see if branch head was updated or not - good for stable
     # tags like 'vX.X' corresponding to releases
 )

 FetchContent_MakeAvailable(sfml)

 FetchContent_MakeAvailable(imgui)

 set(IMGUI_DIR ${imgui_SOURCE_DIR})
 option(IMGUI_SFML_FIND_SFML "Use find_package to find SFML" OFF)
 option(IMGUI_SFML_IMGUI_DEMO "Build imgui_demo.cpp" ON)

 FetchContent_MakeAvailable(imgui-sfml)

 set_target_properties(sfml-graphics sfml-audio sfml-system PROPERTIES CXX_STANDARD 98 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

