cmake_minimum_required(VERSION 3.8)
project(robobike_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add msg/*.msg to rosidl_generate_interfaces(${PROJECT_NAME} ...) when defined.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
