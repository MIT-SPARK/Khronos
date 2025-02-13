/** -----------------------------------------------------------------------------
 * Copyright (c) 2024 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:      Lukas Schmid <lschmid@mit.edu>, Marcus Abate <mabate@mit.edu>,
 *               Yun Chang <yunchang@mit.edu>, Luca Carlone <lcarlone@mit.edu>
 * AFFILIATION:  MIT SPARK Lab, Massachusetts Institute of Technology
 * YEAR:         2024
 * SOURCE:       https://github.com/MIT-SPARK/Khronos
 * LICENSE:      BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <hydra_visualizer/color/mesh_color_adaptor.h>
#include <khronos/common/common_types.h>
#include <ros/node_handle.h>
#include <tf2_ros/transform_broadcaster.h>

namespace khronos {

// TODO(lschmid): Large parts of this functionality were moved to the hydra_visualizer package.
// Double check this class can safely be removed/replaced in the 4D visualizer.

/**
 * @brief Visualization tool for Khronos meshes in a dynamic scene graph.
 */
class KhronosMeshVisualizer {
 public:
  using Coloring = hydra::MeshColoring::ConstPtr;
  // TODO(lschmid): Currently just use a single color for each object.
  using ObjectColors = std::unordered_map<NodeId, Color>;

  // Config.
  struct Config {
    int verbosity = 0;

    // Queue sizes for publishing.
    int queue_size = 100;

    // Reference frame of all published meshes.
    std::string global_frame_name = "world";
  } const config;

  // Construction.
  KhronosMeshVisualizer(const Config& config, const ros::NodeHandle& nh);

  // Visualization.
  /**
   * @brief Visualize the background mesh and all object meshes in the DSG.
   * @param dsg The dynamic scene graph to visualize.
   * @param background_coloring Optional: Coloring to be applied to the background mesh.
   * @param object_colorings Optional: Coloring to be applied to each object mesh by node id.
   */
  void draw(const DynamicSceneGraph& dsg,
            const Coloring& background_coloring = nullptr,
            const ObjectColors& object_colorings = {});

  /**
   * @brief Visualize the background mesh in the DSG.
   * @param dsg The dynamic scene graph to visualize.
   * @param coloring Optional: Coloring to be applied to the background mesh.
   */
  void drawBackground(const DynamicSceneGraph& dsg, const Coloring& coloring = nullptr);

  /**
   * @brief Visualize all object meshes in the DSG.
   * @param dsg The dynamic scene graph to visualize.
   * @param colorings Optional: Coloring to be applied to each object mesh by node id.
   */
  void drawObjects(const DynamicSceneGraph& dsg, const ObjectColors& colorings = {});

  /**
   * @brief Reset the visualization.
   */
  void reset();

  /**
   * @brief Reset the background mesh.
   */
  void resetBackground();

  /**
   * @brief Reset all object meshes.
   */
  void resetObjects();

 private:
  inline static const std::string kBackgroundNs = "background";

  // ROS.
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::unordered_set<uint64_t> previous_objects_;

  // Helper functions.
  void clearMesh(const std::string& ns);

  void publishMesh(const std_msgs::Header& header,
                   const std::string& ns,
                   const Mesh& mesh,
                   const Coloring& coloring);

  void publishTransform(const std_msgs::Header& header,
                        const KhronosObjectAttributes& attrs,
                        const uint64_t id);

  // Mesh namespace for the visualizer plugin.
  static std::string getNamespace(const uint64_t id);

  // TF frame name.
  static std::string getFrameName(const uint64_t id);
};

void declare_config(KhronosMeshVisualizer::Config& config);

}  // namespace khronos
