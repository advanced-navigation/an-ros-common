# Standalone CMakeLists.txt for the an-ros-common library.
#
# This file exists so the shared C/C++ sources can be compiled and verified
# independently of any ROS 2 workspace (e.g. in CI).  It is NOT used when
# building as part of a colcon workspace — colcon discovers packages via
# package.xml, which is intentionally absent here.
cmake_minimum_required(VERSION 3.14)
project(an_ros_common C CXX)

# --- Standards -----------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Warnings ------------------------------------------------------------------
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

# --- Dependencies --------------------------------------------------------------
# std::thread / std::atomic require the pthreads library on Linux.
find_package(Threads REQUIRED)

# --- Library -------------------------------------------------------------------
add_library(an_ros_common STATIC
  src/an_packet_protocol.c
  src/ins_packets.c
  src/rs232.c
  src/adnav_comms.cpp
  src/adnav_utils.cpp
  src/adnav_ntrip.cpp
)

target_include_directories(an_ros_common PUBLIC include)
target_link_libraries(an_ros_common PUBLIC Threads::Threads)
