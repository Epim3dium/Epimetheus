include(FetchContent)

set(SFML_VERSION 2.6.x)
set(IMGUI_VERSION 1.87)
# set(IMGUI_SFML_VERSION 2.3)

# It's nice to get stripped-down release zips instead of cloning
# a repo and checking out a tag
FetchContent_Declare(
  SFML
  GIT_REPOSITORY "https://github.com/SFML/SFML.git"
  GIT_TAG        2.6.0
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY "https://github.com/ocornut/imgui.git"
  GIT_TAG  v1.90.1
)

# Or by tag...
FetchContent_Declare(
  imgui-sfml
  GIT_REPOSITORY "https://github.com/SFML/imgui-sfml.git"
  GIT_TAG        v2.6
  # GIT_COMMIT 5f54b69b6aee18db846c81633f86f78c2586dded
  # ^ or like this - sometimes it's better because FetchContent won't look
  # into remote to see if branch head was updated or not - good for stable
  # tags like 'vX.X' corresponding to releases
)

# For Windows: Prevent overriding the parent project's compiler/linker settings

# SFML
FetchContent_MakeAvailable(sfml)

# Dear ImGui
FetchContent_MakeAvailable(imgui)

# ImGui-SFML
set(IMGUI_DIR ${imgui_SOURCE_DIR})
option(IMGUI_SFML_FIND_SFML "Use find_package to find SFML" OFF)
option(IMGUI_SFML_IMGUI_DEMO "Build imgui_demo.cpp" ON)

FetchContent_MakeAvailable(imgui-sfml)
