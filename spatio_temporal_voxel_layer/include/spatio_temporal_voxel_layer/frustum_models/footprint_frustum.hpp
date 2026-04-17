#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_FOOTPRINT_FRUSTUM_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_FOOTPRINT_FRUSTUM_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/geometry.hpp>
#include <memory>
#include <optional>
#include <spatio_temporal_voxel_layer/frustum_models/frustum.hpp>

#include "ros/ros.h"
namespace geometry {

/**
 * @brief A 2D polygon-based frustum for filtering points within a robot's
 * footprint.
 *
 * FootprintFrustum defines a 2D polygonal zone that can be transformed
 * (rotated and translated) based on the robot's pose. Points can be tested
 * for inclusion within this polygon, which is useful for filtering obstacle
 * points that fall within the robot's physical footprint.
 */
class FootprintFrustum : public Frustum {
 public:
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Polygon = boost::geometry::model::polygon<Point>;

  /**
   * @brief Configuration parameters for FootprintFrustum.
   */
  struct Config {
    /**
     * @brief Validates the configuration.
     * @return True if the config has at least 3 footprint points, false
     * otherwise.
     */
    bool IsValid() const;

    /**
     * @brief Loads configuration from the ROS parameter server.
     *
     * Expected parameters under the given NodeHandle:
     * - enable (bool): Whether the frustum is enabled
     * - point1, point2, ... (array of 2 doubles): Footprint vertices as [x, y]
     *
     * @param nh The ROS NodeHandle to load parameters from.
     * @return The loaded config, or std::nullopt if 'enable' param is missing.
     */
    static std::optional<Config> Load(ros::NodeHandle& nh);

    /// If false, the footprint frustum is disabled and all points are
    /// considered outside the frustum.
    bool enable = false;
    /// The vertices of the footprint polygon in the robot's local frame.
    std::vector<Point> footprint_points;
  };

  /**
   * @brief Factory method to create a FootprintFrustum instance.
   * @param config The configuration for the frustum.
   * @param nh The ROS NodeHandle for publishing visualizations.
   * @return A unique_ptr to the created FootprintFrustum, or nullptr if the
   * config is invalid.
   */
  static std::unique_ptr<FootprintFrustum> Create(const Config& config,
                                                  ros::NodeHandle nh);

  virtual ~FootprintFrustum();

  /**
   * @brief Sets the footprint polygon vertices.
   * @param points The vertices of the footprint polygon (minimum 3 required).
   * @return True if the footprint was set successfully, false if fewer than 3
   * points provided.
   */
  bool SetFootprint(const std::vector<Point>& points);

  /**
   * @brief Transforms the footprint polygon based on current position and yaw.
   *
   * Applies rotation (by yaw) and translation (by position) to the original
   * footprint polygon to compute the current safety zone polygon in world
   * coordinates.
   */
  void TransformModel() override;

  /**
   * @brief Determines if a point is inside the transformed footprint polygon.
   *
   * Only the x and y components of the point are used; the z component is
   * ignored.
   *
   * @param pt The 3D point to test (only x, y are used).
   * @return True if the point is inside the polygon and the frustum is enabled,
   * false otherwise.
   */
  bool IsInside(const openvdb::Vec3d& pt) override;

  /**
   * @brief Sets the position of the frustum in world coordinates.
   * @param origin The position to set.
   */
  void SetPosition(const geometry_msgs::Point& origin) override;

  /**
   * @brief Sets the orientation of the frustum and extracts yaw for 2D
   * transformation.
   * @param quat The quaternion orientation to set. Yaw is extracted and used
   * by TransformModel().
   */
  void SetOrientation(const geometry_msgs::Quaternion& quat) override;

  /**
   * @brief Populates a marker array for visualizing the frustum.
   * @param msg_list The marker array to populate.
   * @note Currently not implemented (returns immediately).
   */
  void GetVisualizationMarker(visualization_msgs::MarkerArray& msg_list);

  /**
   * @brief Publishes visualization markers for the current footprint polygon.
   *
   * Publishes sphere markers for each vertex and line strips connecting them
   * to the "footprint_frustum_visualization" topic.
   */
  void PublishVisualization();

 private:
  FootprintFrustum(const Config& config, ros::NodeHandle nh);
  Config config_;
  Eigen::Vector3d _position;
  double _yaw;
  double _pitch;
  double _roll;
  Eigen::Quaterniond _orientation;
  Polygon curr_safety_zone_polygon_;
  Polygon original_safety_zone_polygon_;

  ros::Publisher visualization_pub_;
  ros::NodeHandle nh_;
};

}  // namespace geometry

#endif
