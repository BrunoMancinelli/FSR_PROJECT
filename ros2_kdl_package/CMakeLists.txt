cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(aruco REQUIRED)
find_package(tf2_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_library(kdl_robot_utils
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)


target_include_directories(kdl_robot_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(kdl_robot_utils
  rclcpp
  orocos_kdl
  kdl_parser
  Eigen3
  sensor_msgs
  std_msgs
  cv_bridge
  kdl_parser
  image_transport
  aruco
)

target_compile_features(kdl_robot_utils PUBLIC cxx_std_17)

ament_export_targets(kdl_robot_utils_export HAS_LIBRARY_TARGET)
ament_export_libraries(kdl_robot_utils)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
  orocos_kdl
  kdl_parser
  geometry_msgs
  sensor_msgs
  std_msgs
  rclcpp
  tf2_msgs
  image_transport
  aruco
  OpenCV
)
ament_export_targets(${library_name})


install(TARGETS kdl_robot_utils
  EXPORT kdl_robot_utils_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT kdl_robot_utils_export
  FILE kdl_robot_utilsTargets.cmake
  NAMESPACE ros2_kdl_package::
  DESTINATION lib/cmake/ros2_kdl_package
)

  
install(DIRECTORY include/
  DESTINATION include
) 
install(DIRECTORY src/
  DESTINATION src
)  


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
