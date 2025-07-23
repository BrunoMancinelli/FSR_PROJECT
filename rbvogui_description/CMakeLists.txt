cmake_minimum_required(VERSION 3.8)
project(rbvogui_description)

# Usa ament_cmake per ROS 2
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(dynamic_tf_publisher src/odom_bl_tf.cpp)
ament_target_dependencies(dynamic_tf_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(DIRECTORY worlds models config 
  DESTINATION share/${PROJECT_NAME}
)

# Installazione dei file di descrizione robotica
install(DIRECTORY urdf robots meshes
  DESTINATION share/${PROJECT_NAME}
)

# Installazione dei file di lancio
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Installazione dei file di configurazione RViz
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS dynamic_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
