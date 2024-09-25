cmake_minimum_required(VERSION 3.5)
project(moveit_wrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  tf2_eigen
  moveit_core
  moveit_servo
  moveit_ros_planning
  pluginlib
  control_msgs
  geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
# moveit_package()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



set(srv_files
  "srv/MoveToPose.srv"
  "srv/MoveToJointPosition.srv"
  "srv/String.srv"
  "srv/SetVelocity.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

add_library(
  ${PROJECT_NAME}_lib
  SHARED
  src/moveit_wrapper.cpp
)
target_include_directories(
  ${PROJECT_NAME}_lib
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
)
#rosidl_target_interfaces(
#${PROJECT_NAME}_lib
#   ${PROJECT_NAME}
#   "rosidl_typesupport_cpp"
#)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  moveit_core
  moveit_ros_planning_interface
  ${PROJECT_NAME}
  geometry_msgs
)
install(
  TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
#add_executable(servo_server src/servo_node_main.cpp)
#target_include_directories(servo_server PUBLIC include)
#target_link_libraries(servo_server ${PROJECT_NAME})
#ament_target_dependencies(servo_server ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(moveit_wrapper_node src/moveit_wrapper_node.cpp  src/moveit_wrapper.cpp)
target_include_directories(
  moveit_wrapper_node
  PRIVATE
  include)
ament_target_dependencies(moveit_wrapper_node rclcpp rclcpp_lifecycle moveit_core moveit_ros_planning_interface geometry_msgs)

#rosidl_target_interfaces(
#moveit_wrapper_node
#   ${PROJECT_NAME}
#   "rosidl_typesupport_cpp"
#)
target_link_libraries(moveit_wrapper_node ${cpp_typesupport_target})

install(TARGETS
  moveit_wrapper_node #servo_server
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  srv
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
