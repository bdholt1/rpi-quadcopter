cmake_minimum_required(VERSION 3.10)
project(quadcopter_motor_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(quadcopter_motor_controller_node src/quadcopter_motor_controller_node.cpp)
target_include_directories(quadcopter_motor_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(quadcopter_motor_controller_node
        "rclcpp"
        "std_msgs")

target_compile_options(quadcopter_motor_controller_node PRIVATE -Werror)
target_compile_features(quadcopter_motor_controller_node PRIVATE cxx_std_14)

install(TARGETS quadcopter_motor_controller_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
