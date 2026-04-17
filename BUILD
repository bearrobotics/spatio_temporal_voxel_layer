load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")
load("@rules_cc//cc:defs.bzl", "cc_library")
load(
    "//build_rules/ros:defs.bzl",
    "cc_ros_dynamic_reconfigure",
    "cc_ros_interface",
    "cc_ros_test",
    "ros_dynamic_reconfigure",
    "ros_interface",
)

refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        ":all": "",
    },
)

ros_interface(
    name = "spatio_temporal_voxel_layer",
    srcs = glob([
        "msgs/*",
        "srv/*",
        "action/*",
    ]),
    deps = [
        "//third_party/ros:geometry_msgs",
        "//third_party/ros:std_msgs",
    ],
)

cc_ros_interface(
    name = "cc_spatio_temporal_voxel_layer",
    visibility = [
        "//ROS/external/navigation:__subpackages__",
    ],
    deps = [":spatio_temporal_voxel_layer"],
)

ros_dynamic_reconfigure(
    name = "spatio_temporal_voxel_layer_cfg",
    src = "cfg/SpatioTemporalVoxelLayer.cfg",
    pkg_override = "spatio_temporal_voxel_layer",
)

cc_ros_dynamic_reconfigure(
    name = "cc_spatio_temporal_voxel_layer_cfg",
    dep = ":spatio_temporal_voxel_layer_cfg",
)

ros_dynamic_reconfigure(
    name = "noise_filter_cfg",
    src = "cfg/NoiseFilter.cfg",
    pkg_override = "spatio_temporal_voxel_layer",
)

cc_ros_dynamic_reconfigure(
    name = "cc_noise_filter_cfg",
    dep = ":noise_filter_cfg",
)

cc_library(
    name = "stvl_external_build_deps",
    visibility = ["//visibility:public"],
    deps = [
        ":cc_noise_filter_cfg",
        ":cc_spatio_temporal_voxel_layer",
        ":cc_spatio_temporal_voxel_layer_cfg",
        "//ROS/bearlib",
        "//ROS/external/navigation/costmap_2d",
        "//ROS/external/perception_pcl/pcl_ros:pcl_ros_tf",
        "//ROS/multi_robot/public:cc_msgs",
        "//ROS/pennybot_perception/obstacle_detector:cc_obstacle_detector_msgs",
        "//third_party/pcl:common",
        "//third_party/pcl:filters",
        "//third_party/ros:cc_sensor_msgs",
        "//third_party/ros:cc_sensor_msgs_lib",
        "//third_party/ros:cc_visualization_msgs",
        "//third_party/ros:roscpp",
        "@laser_geometry",
        "@openvdb",
        "@ros_comm_msgs//:cc_std_srvs",
        "@ros_geometry2//:tf2_geometry_msgs",
        "@ros_geometry2//:tf2_sensor_msgs",
    ],
)

alias(
    name = "libspatio_temporal_voxel_layer.so",
    actual = "@spatio_temporal_voxel_layer_external//:libspatio_temporal_voxel_layer.so",
    visibility = ["//visibility:public"],
)

alias(
    name = "spatio_temporal_voxel_layer_plugin",
    actual = "@spatio_temporal_voxel_layer_external//:spatio_temporal_voxel_layer_plugin",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "test_utils",
    testonly = True,
    srcs = ["test/test_utils.cpp"],
    hdrs = ["test/test_utils.h"],
    includes = ["."],
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "//third_party/ros:cc_geometry_msgs",
        "//third_party/ros:cc_sensor_msgs",
        "//third_party/ros:cc_sensor_msgs_lib",
        "//third_party/ros:roscpp",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_filter_factory",
    size = "small",
    srcs = ["test/test_filter_factory.cpp"],
    launch_file = "test/test_filter_factory.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@perception_pcl//:pcl_conversions",
    ],
)

cc_ros_test(
    name = "test_footprint_frustum",
    size = "small",
    srcs = ["test/test_footprint_frustum.cpp"],
    launch_file = "test/test_footprint_frustum.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "@gtests//:gtest",
    ],
)

cc_ros_test(
    name = "test_dynamic_obstacle_tracker",
    size = "small",
    srcs = ["test/test_dynamic_obstacle_tracker.cpp"],
    launch_file = "test/test_dynamic_obstacle_tracker.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "//third_party/ros:roscpp",
        "@eigen",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_robot_motion_tracker",
    size = "small",
    srcs = ["test/test_robot_motion_tracker.cpp"],
    launch_file = "test/test_robot_motion_tracker.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "//third_party/ros:roscpp",
        "@eigen",
        "@gtests//:gtest",
    ],
)

cc_ros_test(
    name = "test_circle_frustum",
    size = "small",
    srcs = ["test/test_circle_frustum.cpp"],
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "@eigen",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_polygon_frustum",
    size = "small",
    srcs = ["test/test_polygon_frustum.cpp"],
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "@eigen",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_frustum_factory",
    size = "small",
    srcs = ["test/test_frustum_factory.cpp"],
    launch_file = "test/test_frustum_factory.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
    ],
)

cc_ros_test(
    name = "test_update_last_readings",
    size = "small",
    srcs = ["test/test_update_last_readings.cpp"],
    launch_file = "test/test_update_last_readings.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        ":test_utils",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_is_point_in_frustums",
    size = "small",
    srcs = ["test/test_is_point_in_frustums.cpp"],
    launch_file = "test/test_is_point_in_frustums.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        ":test_utils",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_check_box",
    size = "small",
    srcs = ["test/test_check_box.cpp"],
    launch_file = "test/test_check_box.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        ":test_utils",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_blind_spot_integration",
    size = "small",
    srcs = ["test/test_blind_spot_integration.cpp"],
    launch_file = "test/test_blind_spot_integration.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        ":test_utils",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_check_blind_spot_service",
    size = "small",
    srcs = ["test/test_check_blind_spot_service.cpp"],
    launch_file = "test/test_check_blind_spot_service.test",
    deps = [
        ":cc_spatio_temporal_voxel_layer",
        ":spatio_temporal_voxel_layer_plugin",
        ":test_utils",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@openvdb",
    ],
)

cc_ros_test(
    name = "test_clear_robot_footprint_service",
    size = "small",
    srcs = ["test/test_clear_robot_footprint_service.cpp"],
    launch_file = "test/test_clear_robot_footprint_service.test",
    deps = [
        ":spatio_temporal_voxel_layer_plugin",
        ":test_utils",
        "//third_party/ros:roscpp",
        "@gtests//:gtest",
        "@openvdb",
    ],
)
