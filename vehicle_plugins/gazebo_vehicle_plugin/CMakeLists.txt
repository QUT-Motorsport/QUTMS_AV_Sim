cmake_minimum_required(VERSION 3.5)

add_library(gazebo_vehicle SHARED
  src/gazebo_vehicle.cpp)

ament_target_dependencies(gazebo_vehicle
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "nav_msgs"
  "tf2"
  "ackermann_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_srvs"
  "std_msgs"
  "sensor_msgs"
  "yaml-cpp"
  "geometry_msgs"
)

target_include_directories(gazebo_vehicle PUBLIC include)
target_include_directories(gazebo_vehicle PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(gazebo_vehicle)
target_link_libraries(gazebo_vehicle
  ${YAML_CPP_LIBRARIES})
ament_export_libraries(gazebo_vehicle)

install(TARGETS gazebo_vehicle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
