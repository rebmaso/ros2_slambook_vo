add_executable(main vo_node.cpp)
target_link_libraries(main myslam ${THIRD_PARTY_LIBS})
ament_target_dependencies(main rclcpp std_msgs sensor_msgs message_filters)