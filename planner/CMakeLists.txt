cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === TROVA LE DIPENDENZE ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(aruco REQUIRED)
find_package(ros2_kdl_package REQUIRED)
find_package(ros2_opencv REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# === AGGIUNGI IL NODO C++ ===
add_executable(bicycle_tracker_node src/ros2_node.cpp)


target_link_libraries(bicycle_tracker_node Eigen3::Eigen)

ament_target_dependencies(bicycle_tracker_node
  controller_manager_msgs
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  orocos_kdl
  kdl_parser
  image_transport
  OpenCV
  aruco
  cv_bridge
  ros2_opencv
  ros2_kdl_package
)

# === INSTALLA L'ESEGUIBILE ===
install(TARGETS
  bicycle_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/trajectory_cart.csv
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  src/trajectory_circ.csv
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  src/trajectory_cart_straight.csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
