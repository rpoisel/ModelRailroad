add_executable(LadderTrack main.cpp LadderTrack.cpp Arduino.cpp Wire.cpp)
target_include_directories(LadderTrack PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
