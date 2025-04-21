cmake_minimum_required(VERSION 3.5)
project(athena_description)

# Find necessary packages
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  urdf
  meshes
  ros2_control
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(hooks/${PROJECT_NAME}.dsv.in)
ament_environment_hooks(hooks/${PROJECT_NAME}.sh.in)

# Mark the package as an ament package
ament_package()
