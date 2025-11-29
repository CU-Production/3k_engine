# sokol
add_library(sokol INTERFACE)
target_include_directories(sokol INTERFACE sokol)

# hmm
add_library(hmm INTERFACE)
target_include_directories(hmm INTERFACE hmm)

# imgui
add_library(imgui
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp)
target_include_directories(imgui PUBLIC imgui)

# lua headers
add_library(lua lua/onelua.c)
target_include_directories(lua PUBLIC lua)

# sol2 (header-only)
add_library(sol2 INTERFACE)
target_include_directories(sol2 INTERFACE sol2/include)

# stb (header-only)
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE stb)

# box2d (library)
add_subdirectory(box2d)

# physfs (library)
add_subdirectory(physfs)
