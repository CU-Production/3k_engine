cmake_minimum_required(VERSION 4.0)
project(simple2dengine)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(3rd_party)

add_executable(testsokol main.cpp)
target_link_libraries(testsokol PRIVATE sokol hmm imgui box2d PhysFS::PhysFS-static sol2 lua stb nfd)
add_custom_command(
    TARGET testsokol POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ARGS "${CMAKE_CURRENT_SOURCE_DIR}/flappycube" "${CMAKE_CURRENT_BINARY_DIR}/flappycube"
)
