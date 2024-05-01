cmake_minimum_required(VERSION 3.20)
project(minimal)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(mock)
add_executable(LadderTrack src/LadderTrack.cpp)
target_link_libraries(LadderTrack PRIVATE mock)
