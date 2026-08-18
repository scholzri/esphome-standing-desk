cmake_minimum_required(VERSION 3.20)
project(desk_link_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(desk_link_tests
  test_main.cpp
  test_ctb_crc.cpp
  test_frame_codec.cpp
  test_height_codec.cpp
  test_motion_controller.cpp
  ../components/desk_link/frame_codec.cpp
  ../components/desk_link/motion_controller.cpp
)

target_compile_options(desk_link_tests PRIVATE -Wall -Wextra)

enable_testing()
add_test(NAME desk_link_tests COMMAND desk_link_tests)
