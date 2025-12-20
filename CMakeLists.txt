# NOTE(hashir): This CMakeLists.txt is used only for generating messages for
# legacy catkin packages. Bazel BUILD files are used for building the actual
# library and binaries.
# add_message_files(

cmake_minimum_required(VERSION 2.8.3)
project(spatio_temporal_voxel_layer)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
install(
  CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_BINARY_DIR}/compile_commands.json
  ${CMAKE_SOURCE_DIR}/compile_commands.json
)")

# Treat all warnings as errors
add_compile_options(-Wall -Werror)

find_package(catkin REQUIRED
    COMPONENTS
        dynamic_reconfigure
        geometry_msgs
        message_generation
        roscpp
        rospy
        std_srvs
)

add_service_files(
  DIRECTORY srv
  FILES SaveGrid.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/SpatioTemporalVoxelLayer.cfg
    cfg/NoiseFilter.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
