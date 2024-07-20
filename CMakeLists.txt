# cmake_minimum_required(VERSION 3.8)
# project(drone)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(gazebo_msgs REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/apply_force_node.py
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY worlds launch models meshes xacros
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()


















cmake_minimum_required(VERSION 3.8) #maybe 3.5 will work if there is an error.
project(drone)



##############################
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
##############################


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
# find_package(ignition-math6 REQUIRED)

# find_package(xacro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
# include_directories(${GAZEBO_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${ignition-math6_INCLUDE_DIRS})



###for ros node
add_executable(apply_force_node src/apply_force_node.cpp)           #https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
ament_target_dependencies(apply_force_node rclcpp geometry_msgs gazebo_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


###for ros node
install(TARGETS apply_force_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY worlds launch models meshes xacros
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
