#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_FOOTPRINT_CLEARING_PRISM_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_FOOTPRINT_CLEARING_PRISM_HPP_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_models/clearing_frustum.h"
#include "visualization_msgs/MarkerArray.h"

namespace geometry {

/**
 * @brief A robot-anchored polygonal prism for unconditional voxel clearing
 * in a bounded near-field region.
 *
 * FootprintClearingPrism defines a 3D prism (2D polygon extruded between
 * min_z and max_z) that tracks the robot's pose. Voxels inside the prism
 * are unconditionally cleared, which prevents stale obstacle memory in
 * sensor blind spots close to the robot body.
 */
class FootprintClearingPrism : public IClearingFrustum {
 public:
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Polygon = boost::geometry::model::polygon<Point>;

  /**
   * @brief Configuration parameters for FootprintClearingPrism.
   */
  struct Config {
    /**
     * @brief Loads configuration from the ROS parameter server.
     *
     * Expected parameters under the given NodeHandle:
     * - enable (bool): Whether the prism is enabled.
     * - publish_visualization (bool, optional): Publish markers for RViz.
     * - min_z (double): Inclusive lower height bound (meters).
     * - max_z (double): Inclusive upper height bound (meters).
     * - point1, point2, ... (array of 2 doubles): Polygon vertices as [x, y]
     *   in the robot's base_footprint frame.
     *
     * @param nh The ROS NodeHandle to load parameters from.
     * @return The loaded config, or std::nullopt on error.
     */
    static std::optional<Config> Load(ros::NodeHandle& nh);

    /**
     * @brief Validates the configuration.
     * @return True if the config is valid (disabled, or has >= 3 points and
     *         max_z > min_z).
     */
    bool IsValid() const;

    bool enable = false;
    bool publish_visualization = true;
    double min_z = 0.0;
    double max_z = 0.0;
    std::vector<Point> footprint_points;
  };

  /**
   * @brief Factory method to create a FootprintClearingPrism instance.
   * @param config The configuration for the prism.
   * @param nh The ROS NodeHandle for publishing visualizations.
   * @return A unique_ptr to the created prism, or nullptr if the config is
   *         invalid.
   */
  static std::unique_ptr<FootprintClearingPrism> Create(const Config& config,
                                                        ros::NodeHandle nh);

  ~FootprintClearingPrism() override;

  /**
   * @brief Sets the polygon vertices for the clearing region.
   * @param points The vertices (minimum 3 required).
   * @return True on success, false if fewer than 3 points.
   */
  bool SetFootprint(const std::vector<Point>& points);

  /**
   * @brief Sets the position of the prism in world coordinates.
   * @param origin The robot's current position.
   */
  void SetPosition(const geometry_msgs::Point& origin);

  /**
   * @brief Sets the orientation and extracts yaw for 2D transformation.
   * @param quat The robot's current orientation quaternion.
   */
  void SetOrientation(const geometry_msgs::Quaternion& quat);

  /**
   * @brief Applies rotation and translation to the polygon.
   *
   * Must be called after SetPosition() and SetOrientation() to update
   * the polygon in world coordinates.
   */
  void TransformModel();

  /**
   * @brief Tests whether a 3D point falls inside the prism.
   * @param point The point to test (x, y for polygon, z for height bounds).
   * @return True if the point is inside the prism and the prism is enabled.
   */
  [[nodiscard]] bool IsInside(const openvdb::Vec3d& point) const override;

  /**
   * @brief Publishes RViz markers showing the prism edges.
   */
  void PublishVisualization() const;

 private:
  FootprintClearingPrism(const Config& config, ros::NodeHandle nh);

  Config config_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  double yaw_ = 0.0;
  Polygon original_polygon_;
  Polygon current_polygon_;

  ros::NodeHandle nh_;
  ros::Publisher visualization_pub_;
};

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_FOOTPRINT_CLEARING_PRISM_HPP_
