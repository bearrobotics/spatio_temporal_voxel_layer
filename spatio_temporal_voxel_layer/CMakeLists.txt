cmake_minimum_required(VERSION 3.0.2)
project(spatio_temporal_voxel_layer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    laser_geometry
    message_generation
    multi_robot_public
    obstacle_detector
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tf2_sensor_msgs
    visualization_msgs
)

find_package(OpenVDB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_service_files(FILES
    SaveGrid.srv
    CheckBlindSpot.srv
)

generate_dynamic_reconfigure_options(
    cfg/SpatioTemporalVoxelLayer.cfg
    cfg/NoiseFilter.cfg
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        message_runtime
        multi_robot_public
        obstacle_detector
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        std_srvs
        tf2
        tf2_geometry_msgs
        tf2_ros
        tf2_sensor_msgs
        visualization_msgs
    DEPENDS
        OpenVDB
        PCL
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenVDB_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/spatio_temporal_voxel_layer.cpp
    src/spatio_temporal_voxel_grid.cpp
    src/measurement_buffer.cpp
    src/dynamic_obstacle_tracker.cpp
    src/robot_motion_tracker.cpp
    src/filter_factory.cpp
    src/frustum_factory.cpp
    src/noise_filter.cpp
    src/vdb2pc.cpp
    src/frustum_models/depth_camera_frustum.cpp
    src/frustum_models/footprint_frustum.cpp
    src/frustum_models/three_dimensional_lidar_frustum.cpp
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenVDB_LIBRARIES}
    ${PCL_LIBRARIES}
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/ include/bearlib/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY example/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/example
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_gtest(test_circle_frustum test/test_circle_frustum.cpp)
    target_link_libraries(test_circle_frustum ${PROJECT_NAME} ${catkin_LIBRARIES})

    catkin_add_gtest(test_polygon_frustum test/test_polygon_frustum.cpp)
    target_link_libraries(test_polygon_frustum ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_filter_factory test/test_filter_factory.test test/test_filter_factory.cpp)
    target_link_libraries(test_filter_factory ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_footprint_frustum test/test_footprint_frustum.test test/test_footprint_frustum.cpp)
    target_link_libraries(test_footprint_frustum ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_dynamic_obstacle_tracker test/test_dynamic_obstacle_tracker.test test/test_dynamic_obstacle_tracker.cpp)
    target_link_libraries(test_dynamic_obstacle_tracker ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_robot_motion_tracker test/test_robot_motion_tracker.test test/test_robot_motion_tracker.cpp)
    target_link_libraries(test_robot_motion_tracker ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_frustum_factory test/test_frustum_factory.test test/test_frustum_factory.cpp)
    target_link_libraries(test_frustum_factory ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_update_last_readings test/test_update_last_readings.test test/test_update_last_readings.cpp test/test_utils.cpp)
    target_link_libraries(test_update_last_readings ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_is_point_in_frustums test/test_is_point_in_frustums.test test/test_is_point_in_frustums.cpp test/test_utils.cpp)
    target_link_libraries(test_is_point_in_frustums ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_check_box test/test_check_box.test test/test_check_box.cpp test/test_utils.cpp)
    target_link_libraries(test_check_box ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_blind_spot_integration test/test_blind_spot_integration.test test/test_blind_spot_integration.cpp test/test_utils.cpp)
    target_link_libraries(test_blind_spot_integration ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_check_blind_spot_service test/test_check_blind_spot_service.test test/test_check_blind_spot_service.cpp test/test_utils.cpp)
    target_link_libraries(test_check_blind_spot_service ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(test_clear_robot_footprint_service test/test_clear_robot_footprint_service.test test/test_clear_robot_footprint_service.cpp test/test_utils.cpp)
    target_link_libraries(test_clear_robot_footprint_service ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
