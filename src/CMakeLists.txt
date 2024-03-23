add_executable(main
        vo_node.cpp
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        loopclos.cpp)

target_link_libraries(main
        ${THIRD_PARTY_LIBS})

ament_target_dependencies(main rclcpp std_msgs sensor_msgs message_filters)