cmake_minimum_required(VERSION 3.5)
project(altitude_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(altitude_controller_node src/altitude_controller_node.cpp)
target_include_directories(altitude_controller_node
        PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(altitude_controller_node
        "Eigen3"
        "rclcpp"
        "sensor_msgs"
        "std_msgs")

target_compile_options(altitude_controller_node PRIVATE -Werror)
target_compile_features(altitude_controller_node PRIVATE cxx_std_14)

install(TARGETS altitude_controller_node
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
