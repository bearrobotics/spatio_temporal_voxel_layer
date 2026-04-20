# Spatio-Temporal Voxel Layer

A ROS costmap plugin that uses [OpenVDB](http://www.openvdb.org/) to maintain a 3D voxel grid with temporal decay for obstacle persistence. Originally developed by [Steve Macenski](https://github.com/SteveMacenski) at Simbe Robotics, this fork is maintained by [Bear Robotics](https://www.bearrobotics.ai/).

## Repository Structure

| Package | Description |
|---------|-------------|
| `spatio_temporal_voxel_layer/` | The STVL costmap plugin |
| `multi_robot_public_msgs/` | ROS message definitions used by STVL for multi-robot coordination |
| `obstacle_detector_msgs/` | ROS message definitions used by STVL for obstacle detection |

## Building

### Prerequisites

- ROS Noetic
- OpenVDB (`sudo apt-get install libopenvdb-dev`)
- Eigen3 (`sudo apt-get install libeigen3-dev`)
- PCL (`sudo apt-get install libpcl-dev`)

### Build from Source

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/bearrobotics/spatio_temporal_voxel_layer.git
cd ~/catkin_ws
catkin_make
```

## Usage

Add the plugin to your costmap configuration:

```yaml
plugins:
  - {name: stvl_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}

stvl_layer:
  enabled: true
  voxel_decay: 15
  decay_model: 0
  voxel_size: 0.05
  track_unknown_space: true
  mark_threshold: 0
  update_footprint_enabled: true
  combination_method: 1
  origin_z: 0.0
  publish_voxel_map: false
  transform_tolerance: 0.2
  mapping_mode: false
  observation_sources: rgbd1_mark rgbd1_clear
```

See `spatio_temporal_voxel_layer/example/` for full configuration examples.

## License

This project is licensed under the [LGPL v2.1](spatio_temporal_voxel_layer/LICENSE).

## Citation

```bibtex
@article{doi:10.1177/1729881420910530,
    author = {Steve Macenski and David Tsai and Max Feinberg},
    title = {Spatio-temporal voxel layer: A view on robot perception for the dynamic world},
    journal = {International Journal of Advanced Robotic Systems},
    volume = {17},
    number = {2},
    year = {2020},
    doi = {10.1177/1729881420910530}
}
```
