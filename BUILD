load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")
load(
    "//build_rules/ros:defs.bzl",
    "cc_ros_dynamic_reconfigure",
    "cc_ros_interface",
    "cc_ros_test",
    "ros_dynamic_reconfigure",
    "ros_interface",
    "ros_plugin",
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
        "//third_party/ros:std_msgs",
    ],
)

cc_ros_interface(
    name = "cc_spatio_temporal_voxel_layer",
    visibility = [],
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

ros_plugin(
    name = "spatio_temporal_voxel_layer_plugin",
    srcs = [
        "src/dynamic_obstacle_tracker.cpp",
        "src/filter_factory.cpp",
        "src/frustum_models/depth_camera_frustum.cpp",
        "src/frustum_models/footprint_frustum.cpp",
        "src/frustum_models/three_dimensional_lidar_frustum.cpp",
        "src/measurement_buffer.cpp",
        "src/noise_filter.cpp",
        "src/spatio_temporal_voxel_grid.cpp",
        "src/spatio_temporal_voxel_layer.cpp",
        "src/vdb2pc.cpp",
    ],
    hdrs = glob([
        "include/spatio_temporal_voxel_layer/**/*.h",
        "include/spatio_temporal_voxel_layer/**/*.hpp",
    ]),
    includes = ["include"],
    plugin_file = "costmap_plugins.xml",
    visibility = ["//ROS/external/navigation:__subpackages__"],
    deps = [
        ":cc_noise_filter_cfg",
        ":cc_spatio_temporal_voxel_layer",
        ":cc_spatio_temporal_voxel_layer_cfg",
        "//ROS/bearlib",
        "//ROS/external/navigation/costmap_2d",
        "//ROS/external/perception_pcl/pcl_ros:pcl_ros_tf",
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
