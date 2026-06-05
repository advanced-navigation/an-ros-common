# CI stub — validates msg/srv definitions as a minimal adnav_interfaces package.
# See .github/workflows/ci.yml for how this is assembled and built.
cmake_minimum_required(VERSION 3.8)
project(adnav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  msg/ANPPHeader.msg
  msg/DeviceInformationPacket.msg
  msg/FilterStatus.msg
  msg/GNSSFixStatus.msg
  msg/LLH.msg
  msg/NED.msg
  msg/PacketPeriod.msg
  msg/RPH.msg
  msg/RawAcknowledge.msg
  msg/RawStatusPacket.msg
  msg/RequestPacket.msg
  msg/SerialInterface.msg
  msg/SystemStatus.msg
  # Services
  srv/Ntrip.srv
  srv/PacketPeriods.srv
  srv/PacketTimerPeriod.srv
  srv/RequestPackets.srv
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
