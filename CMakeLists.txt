cmake_minimum_required(VERSION 3.5)

# Declare the project
project(ar_utils)

# Set policy to suppress warnings
cmake_policy(SET CMP0130 NEW)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Install Python packages
# ament_python_install_package()

# Install the Python script node
install(PROGRAMS
  scripts/move_ar.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install service files
# install(DIRECTORY srv/
#   DESTINATION share/${PROJECT_NAME}/srv
# )

# Create an ament package
ament_package()
