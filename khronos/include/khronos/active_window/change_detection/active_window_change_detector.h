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

#include <filesystem>
#include <optional>
#include <unordered_map>
#include <vector>

#include <hydra/common/output_sink.h>
#include <hydra/utils/logging.h>

#include "khronos/active_window/active_window.h"
#include "khronos/active_window/change_detection/transformation_getter.h"
#include "khronos/utils/icp_registration_utils.h"

namespace khronos {

class ActiveWindowChangeDetector : public ActiveWindow::KhronosSink {
 public:
  /**
   * @brief Per-object temporal state for the removed-object EMA filter.
   * Maintained across frames in removed_object_states_.
   */
  struct RemovedObjectState {
    //! EMA-filtered probability that the object's space is free (i.e. object is removed).
    double free_probability = 0.0;
    //! Number of frames for which a valid measurement (num_known > 0) was available.
    int num_frames_observed = 0;
    //! Raw free ratio from the most recent valid measurement (for logging/debugging).
    float last_free_ratio = 0.0f;
  };

  /**
   * @brief Per-track temporal state for the newly-added-object EMA filter.
   * Keyed by track ID (monotonically increasing, never reused by MaxIoUTracker).
   * Caches a full Track snapshot so downstream sinks can access bbox/semantics/confidence
   * even after the track has left the active window.
   */
  struct AddedObjectState {
    //! EMA-filtered probability the track is a newly-added object.
    double added_probability = 0.0;
    //! Number of frames for which a valid containment measurement was available.
    int num_frames_observed = 0;
    //! Raw containment ratio from the most recent valid measurement (for logging/debugging).
    float last_containment = 0.0f;
    //! Latched true once the track passes the added gate; prevents pruning of confirmed objects.
    bool ever_added = false;
    //! frame_index_ value at the last frame this track was measured (for staleness-based pruning).
    int last_updated_frame = 0;
    //! Full Track snapshot, refreshed each frame the track is in the active window.
    Track last_track;
  };

  using ActiveWindowCDSink = hydra::OutputSink<const DynamicSceneGraph::Ptr&,
                                               const std::vector<spark_dsg::NodeId>&,
                                               const std::vector<Track>&,
                                               const Eigen::Isometry3d&>;

  // Config.
  struct Config : hydra::VerbosityConfig {
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    //! Path to prior map to use for change detection.
    std::filesystem::path prior_map_path;

    //! Ratio of free prior map points of an object's vertices in a single frame.
    //! Used as raw input to the EMA filter; not the final removal decision threshold.
    float removal_vertex_free_ratio_threshold = 0.8f;

    //! EMA smoothing factor for the per-object free_probability filter (0 < alpha <= 1).
    //! Lower values give more smoothing (slower to react); higher values track raw ratios closely.
    //! Cumulative running average (alpha=1/n) is an easy drop-in if EMA proves too noisy.
    float removal_ema_alpha = 0.5f;

    //! Smoothed free_probability threshold above which an object is considered removed.
    float removal_probability_threshold = 0.5f;

    //! Minimum number of frames with valid measurements before an object can be declared removed.
    //! Guards against single-frame spikes on newly-entered objects.
    int removal_min_frames_observed = 3;

    //! Raw per-frame containment ratio fed into the EMA filter for newly-added-object detection.
    //! Not the final decision threshold; that role is played by added_probability_threshold.
    float added_object_containment_threshold = 0.5f;

    //! EMA smoothing factor for the per-track added_probability filter (0 < alpha <= 1).
    //! Lower = smoother / slower to react; higher = tracks raw ratios closely.
    float added_ema_alpha = 0.5f;

    //! Smoothed added_probability threshold above which a track is considered a newly-added object.
    float added_probability_threshold = 0.5f;

    //! Minimum valid measurements before a track can be declared newly-added.
    int added_min_frames_observed = 3;

    //! Frames of no measurement after which a non-added track's state record is pruned.
    //! Tracks that ever crossed the threshold are never pruned.
    int added_prune_after_frames = 50;

    //! Sinks for the change detector output.
    std::vector<ActiveWindowCDSink::Factory> awcd_sinks;

    //! Plugin that provides the odom_T_prior (current_T_prior) transform each frame.
    config::VirtualConfig<TransformationGetter> transformation_getter{
        IdentityTransformationGetter::Config{}};

    //! Enable ICP refinement on the transform returned by transformation_getter.
    bool enable_icp_refinement = false;

    //! If true, invert the roman LC transform before using as ICP initial guess.
    //! Verify at runtime: if ICP delta is > ~2m, flip this flag.
    bool invert_roman_lc_transform = false;

    //! Crop radius around robot (m) for mesh point selection.
    float icp_crop_radius = 10.0f;

    //! small_gicp threads.
    size_t icp_num_threads = 2;

    //! Voxel downsampling resolution (m).
    float icp_downsampling_resolution = 0.2f;

    //! Max ICP correspondence distance (m).
    float icp_max_correspondence_distance = 1.0f;

    //! Min inliers to accept refined transform.
    size_t icp_min_inliers = 50;
    
  } const config;

  // Construction.
  explicit ActiveWindowChangeDetector(const Config& config);
  virtual ~ActiveWindowChangeDetector() = default;

  // Module setup.
  /**
   * @brief Add a sink to the active window. The sink will be called whenever the active window
   * finishes processing a frame.
   * @param sink The sink to add.
   */
  void addKhronosSink(const ActiveWindowCDSink::Ptr& sink);

  /**
   * @brief TODO(multy): documentation
   * @param map The current volumetric map that the active window is building.
   * @param data The current data after processing.
   * @param tracks The current tracks in the active window.
   */
  void call(const FrameData& data, const VolumetricMap& map, const Tracks& tracks) const override;

  

  void loadPriorMap();

  bool isPriorPointFree(const Point& point_in_map, const VolumetricMap& map) const;

  bool isPointKnown(const Point& point_in_map, const VolumetricMap& map) const;

  /**
   * @brief Check if a point is within allocated map bounds (has an allocated block).
   * @param point The point to check in world frame.
   * @param map The volumetric map to check against.
   * @return True if the point is within an allocated block of the map.
   */
  bool isPointInMapBounds(const Point& point, const VolumetricMap& map) const;

  /**
   * @brief Find all prior object nodes whose centroid is within the current volumetric map bounds.
   * @param map The current volumetric map.
   * @return Vector of node IDs for objects within map bounds.
   */
  std::vector<spark_dsg::NodeId> findPriorObjectsInMapBounds(const VolumetricMap& map) const;

  /**
   * @brief Set the transform from prior map frame to current map frame.
   * @param current_T_prior Transform that converts points from prior map frame to current frame.
   */
  void setCurrentToPriorTransform(const Eigen::Isometry3d& current_T_prior) const;

  /**
   * @brief Transform a point from prior map frame to current map frame.
   * @param point_in_prior Point in the prior map's world frame.
   * @return Point transformed to the current map's world frame.
   */
  Point transformPriorToCurrentFrame(const Eigen::Vector3d& point_in_prior) const;

 protected:
  /// Runs small_gicp ICP using current map mesh vs prior DSG background mesh.
  /// Updates current_T_prior_ if ICP converges with sufficient inliers.
  void runIcpRefinement(const FrameData& data,
                        const VolumetricMap& map,
                        const Eigen::Isometry3d& initial) const;

  /**
   * @brief Compute the per-frame free ratio for each candidate object with a valid measurement
   * (num_known_vertices > 0). Objects with an empty mesh or no KhronosObjectAttributes are skipped.
   * @return Map from NodeId to raw free ratio [0, 1] for objects that had a valid measurement.
   */
  std::unordered_map<spark_dsg::NodeId, float> computeFreeRatios(
      const std::vector<spark_dsg::NodeId>& objects_id_in_bounds,
      const VolumetricMap& map) const;

  /**
   * @brief Update the EMA filter state for each object that has a measurement this frame,
   * then return the set of objects whose smoothed free_probability passes the removal gate
   * (probability >= removal_probability_threshold && frames >= removal_min_frames_observed).
   * Objects not in measurements are left untouched (freeze-last policy).
   */
  std::vector<spark_dsg::NodeId> updateRemovedFilter(
      const std::unordered_map<spark_dsg::NodeId, float>& measurements) const;

  /**
   * @brief Compute the per-frame containment ratio for each eligible track (non-dynamic,
   * non-empty footprint). Returns a map from track ID to raw containment ratio [0, 1].
   * Reuses getPriorFreeFootprint2D and getTrackFootprint2D.
   */
  std::unordered_map<int, float> computeContainmentRatios(const Tracks& tracks,
                                                          const VolumetricMap& map) const;

  /**
   * @brief Update the EMA filter state for each measured track, refresh cached Track snapshots,
   * prune stale non-added records, then return the filtered set of Track snapshots for tracks
   * that pass the added gate (added_probability >= threshold && frames >= min_frames_observed).
   * Tracks absent from measurements are frozen (cached state and snapshot kept as-is).
   */
  std::vector<Track> updateAddedFilter(const std::unordered_map<int, float>& measurements,
                                       const Tracks& tracks) const;

  // Returns 2D voxel indices (z forced to 0, in current frame) covering the prior
  // traversable (MESH_PLACES / TravNodeAttributes) footprint within map bounds.
  GlobalIndexSet getPriorFreeFootprint2D(const VolumetricMap& map) const;

  // Returns 2D voxel indices (z forced to 0) of a track's last_points footprint.
  GlobalIndexSet getTrackFootprint2D(const Track& track, float voxel_size) const;

  // Convert a 3-D point to a 2D voxel GlobalIndex (z component set to 0).
  static GlobalIndex to2DIndex(const Point& point, float voxel_size_inv);

 private:
  //! Prior map as a 3D scene graph.
  DynamicSceneGraph::Ptr prior_graph_;

  //! Transform from prior map frame to current map frame.
  //! current_point = current_T_prior_ * prior_point
  mutable Eigen::Isometry3d current_T_prior_ = Eigen::Isometry3d::Identity();

  //! Per-object EMA filter state for removed-object detection. Keyed by prior DSG NodeId.
  //! Populated lazily on first observation; never erased (freeze-last for unobserved objects).
  mutable std::unordered_map<spark_dsg::NodeId, RemovedObjectState> removed_object_states_;

  //! Per-track EMA filter state for newly-added-object detection. Keyed by Track::id (monotonic).
  //! Pruned for non-added tracks that go stale; confirmed-added records are kept forever.
  mutable std::unordered_map<int, AddedObjectState> added_object_states_;

  //! Frame counter incremented once per call(), used for staleness-based pruning of added records.
  mutable int frame_index_ = 0;

  //! Plugin that provides the odom_T_prior transform.
  TransformationGetter::Ptr transformation_getter_;

  //! Sinks for the change detector output.
  ActiveWindowCDSink::List sinks_;
};

void declare_config(ActiveWindowChangeDetector::Config& config);

}  // namespace khronos
