cmake_minimum_required(VERSION 4.0)
project(simple2dengine)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(3rd_party)

add_executable(testsokol main.cpp)
target_link_libraries(testsokol PRIVATE sokol hmm imgui box2d PhysFS::PhysFS-static sol2 lua stb)
