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

#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <hydra/common/global_info.h>

#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Map structure that allows queries in space and time.
 */
class SpatioTemporalMap {
 public:
  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // If true finalize the raw data as it arrives.
    bool finalize_incrementally = true;
  } const config;

  // Construction.
  explicit SpatioTemporalMap(const Config& config);
  virtual ~SpatioTemporalMap() = default;

  // Note that these currently create a shallow copy of the DSGs.
  SpatioTemporalMap(const SpatioTemporalMap& other);
  SpatioTemporalMap& operator=(const SpatioTemporalMap& other);
  SpatioTemporalMap(SpatioTemporalMap&& other) noexcept;
  SpatioTemporalMap& operator=(SpatioTemporalMap&& other) noexcept;

  // Interaction.
  /**
   * @brief Update the tracked state of the map. This should be called after loop closures when
   * major updates happen.
   * @param dsg The new state of the dynamic scene graph.
   * @param stamp The time of the update.
   */
  void update(const DynamicSceneGraph::Ptr& dsg, TimeStamp stamp);

  /**
   * @brief Finalize the map by computing an efficient to lookup structure of the current data.
   */
  void finalize();

  // Query the map at a given robot time.
  TimeStamp earliest() const { return earliest_; }
  TimeStamp latest() const { return latest_; }
  TimeStamp current() const { return current_time_; }

  /**
   * @brief Get the scene graph containing all information available up to the given robot time.
   * @param robot_time The time of the query.
   */
  const DynamicSceneGraph& getDsg(TimeStamp robot_time);
  DynamicSceneGraph::Ptr getDsgPtr(TimeStamp robot_time);

  // I/O.
  /**
   * @brief Save the map to a binary file.
   * @param filepath Full path to save the file to.
   */
  bool save(std::string filepath) const;

  /**
   * @brief Load the map from a binary file.
   * @param filepath Full path to load the file from.
   */
  static std::unique_ptr<SpatioTemporalMap> load(std::string filepath);

  // Access to meta data.
  size_t numTimeSteps() const { return stamps_.size(); }
  const std::vector<TimeStamp>& stamps() const { return stamps_; }

 private:
  // Store the state of the DSG at each major update.
  // TODO(lschmid): Replace this with the incremental version.
  std::vector<TimeStamp> stamps_;
  std::vector<DynamicSceneGraph::Ptr> dsgs_;

  // TODO(lschmid): Update for better handling of different robots in the future.
  inline static const hydra::RobotPrefixConfig robot_prefix_;
  inline static const std::string kExtension = ".4dmap";
  inline static const int kSerializationVersion = 1;

  // Time limits the DSG has information about.
  TimeStamp earliest_ = std::numeric_limits<TimeStamp>::max();
  TimeStamp latest_ = 0;

  // Duration up to which information of the robot is considered in the current.
  TimeStamp current_time_ = 0;
  TimeStamp previous_time_ = 0;
  size_t current_dsg_idx_ = std::numeric_limits<size_t>::max();

  // State of the DSG for the given query and robot time.
  DynamicSceneGraph::Ptr current_dsg_;

  // Whether all data is finalized.
  bool finalized_;

  // Index of the last required vertex for each mesh face in each dsg.
  // std::vector<std::vector<size_t>> mesh_face_indices_;

 protected:
  // Construcion helpers.
  void copyMembers(const SpatioTemporalMap& other);
  void moveMembers(SpatioTemporalMap&& other);

  // Finalization.
  void finalizeDsg(DynamicSceneGraph& dsg);
  void finalizeMesh(Mesh& mesh);
  void updateTimingInfo(const DynamicSceneGraph& dsg);

  // Dsg Extraction. TODO: Move this into the incremental version.
  void moveMeshForward();
  void moveAgentForward();
  void moveObjectsForward();
  void moveDynamicObjectAttributesForward();
  void moveMeshBackward();
  void moveAgentBackward();
  void moveObjectsBackward();
  void moveDynamicObjectAttributesBackward();

  std::string printTime(TimeStamp time) const {
    std::stringstream ss;
    ss << time << " (" << std::setprecision(2) << std::fixed << (time - earliest_) / 1e9 << "s)";
    return ss.str();
  }
};

/**
 * @brief Return the new indices of the values after sorting. Result[new_index] = old_index.
 */
std::vector<size_t> sortIndices(const std::vector<uint64_t>& values);

void declare_config(SpatioTemporalMap::Config& config);

}  // namespace khronos
