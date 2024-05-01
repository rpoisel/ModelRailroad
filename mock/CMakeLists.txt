add_library(mock STATIC main.cpp Arduino.cpp Wire.cpp)
target_include_directories(mock PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_compile_options(mock PUBLIC -Wall -Wextra -Wpedantic -Werror)
