add_executable(vo_node vo_node.cpp)
target_link_libraries(vo_node myslam ${THIRD_PARTY_LIBS})
ament_target_dependencies(vo_node rclcpp std_msgs sensor_msgs message_filters)