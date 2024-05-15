cmake_minimum_required(VERSION 3.8)
project(mage_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

set(msg_files
  "msg/AdvancedLogicalCameraImage.msg"
  "msg/Part.msg"
  "msg/PartPose.msg"
  "msg/Sensors.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
)

set(stdsrv_files 
  "std_srvs/Trigger.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${stdsrv_files}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  std_srvs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
