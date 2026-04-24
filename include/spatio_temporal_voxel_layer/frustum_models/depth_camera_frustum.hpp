/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: Structure for handling camera FOVs to construct frustums
 *          and associated methods
 *********************************************************************/
/*
 * --- BEAR MODIFICATION START ---
 * Portions Copyright (c) 2025-2026, Bear Robotics, Inc.
 * This file was modified by Bear Robotics, Inc. between 2025 and 2026.
 * Description of changes:
 *  - Integration with Bear Robotics internal navigation stack
 *  - Multi-robot obstacle tracking and coordination support
 *  - Robot motion tracking for self-clearing
 *  - Front blind-spot clearing prism for near-range obstacle clearing
 *  - CheckBlindSpot and ClearRobotFootprint services
 *  - Sensor data filtering (noise filter, frustum-based filtering)
 *  - Safety zone frustum support
 *  - Various bug fixes and performance improvements
 *    (see git history for detailed per-commit changes)
 * Contributors:
 *  - Vincent Benenati (vincent.benenati@bearrobotics.ai)
 * --- BEAR MODIFICATION END ---
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#ifndef DEPTH_FRUSTUM_H_
#define DEPTH_FRUSTUM_H_

#include <optional>
// STVL
#include <spatio_temporal_voxel_layer/frustum_models/frustum.hpp>

namespace geometry {

// visualize the frustum should someone other than me care
#define VISUALIZE_FRUSTUM true

// A class to model a depth sensor frustum in world space
class DepthCameraFrustum : public Frustum {
 public:
  struct Config {
    /**
     * @brief Loads DepthCameraFrustum configuration from ROS parameters.
     *
     * Required parameters:
     * - vertical_fov_angle (double): Vertical field of view in radians.
     * - horizontal_fov_angle (double): Horizontal field of view in radians.
     * - min_z (double): Minimum sensing distance.
     * - max_z (double): Maximum sensing distance.
     *
     * @param nh ROS NodeHandle with frustum parameters.
     * @return Config if all parameters loaded successfully, std::nullopt
     *         otherwise.
     */
    static std::optional<Config> Load(ros::NodeHandle& nh);

    double vertical_fov_angle = 0.0;
    double horizontal_fov_angle = 0.0;
    double min_distance = 0.0;
    double max_distance = 0.0;
  };

  DepthCameraFrustum(Config config);
  virtual ~DepthCameraFrustum(void);

  // transform plane normals by depth camera pose
  virtual void TransformModel(void);

  // determine if a point is inside of the transformed frustum
  virtual bool IsInside(const openvdb::Vec3d& pt);

  // set pose of depth camera in global space
  virtual void SetPosition(const geometry_msgs::Point& origin);
  virtual void SetOrientation(const geometry_msgs::Quaternion& quat);
  void GetVisualizationMarker(
      visualization_msgs::MarkerArray& msg_list) override;

 private:
  // utils to find useful frustum metadata
  void ComputePlaneNormals(void);
  double Dot(const VectorWithPt3D&, const openvdb::Vec3d&) const;
  double Dot(const VectorWithPt3D&, const Eigen::Vector3d&) const;

  double _vFOV, _hFOV, _min_d, _max_d;
  std::vector<VectorWithPt3D> _plane_normals;
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
  bool _valid_frustum;

#if VISUALIZE_FRUSTUM
  std::vector<Eigen::Vector3d> _frustum_pts;
  visualization_msgs::MarkerArray msg_list_;
#endif
};

}  // namespace geometry

#endif
