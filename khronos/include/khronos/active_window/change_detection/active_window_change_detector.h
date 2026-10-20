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
#include <set>
#include <unordered_map>
#include <vector>

#include <hydra/common/output_sink.h>
#include <hydra/utils/logging.h>
#include <spatial_hash/hash.h>

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
    //! Sensor frame time (ns) the object first crossed the removal gate. Latched once set;
    //! never reset even if the object later drops out of the removed set and returns.
    //! 0 = not yet declared removed.
    TimeStamp first_removed_ns = 0;
  };

  /**
   * @brief A removed object paired with the sensor frame time it was first declared removed.
   * The timestamp is latched at the detector (see RemovedObjectState::first_removed_ns) so it
   * stays constant across every message/frame that reports this object as removed.
   */
  struct RemovedObject {
    spark_dsg::NodeId id;
    TimeStamp first_removed_ns;
    //! Change-detection confidence: EMA-smoothed free-space probability
    //! (RemovedObjectState::free_probability) that drove the removal gate. The removal-side
    //! analogue of AddedObject::change_confidence -- same role, different measurement.
    float change_confidence = 0.0f;
    //! RemovedObjectState::num_frames_observed at the time this object was reported.
    int num_frames_observed = 0;
  };

  /**
   * @brief Per-track added-filter state: containment EMA plus a snapshot of the track, kept and
   * reported after the track is gone once added.
   */
  struct TrackAddedState {
    //! EMA-filtered probability that this track is a newly-added object (raw input: free-space
    //! containment of the track's footprint).
    double change_confidence = 0.0;
    //! Number of frames for which a valid containment measurement was available.
    int num_frames_observed = 0;
    //! Raw containment ratio from the most recent valid measurement (for logging/debugging).
    float last_containment = 0.0f;
    //! frame_index_ value at the last frame this track was measured (staleness pruning).
    int last_updated_frame = 0;
    //! Snapshot of the track at its last measurement.
    BoundingBox bounding_box;
    std::optional<SemanticClusterInfo> semantics;
    TimeStamp first_seen = 0;
    TimeStamp last_seen = 0;
    //! Track::confidence (tracking quality).
    float confidence = 0.0f;
    //! Latched true once the track passes the added gate; prevents pruning.
    bool ever_added = false;
    //! The track id and the ids of all tracks it absorbed (Track::absorbed_track_ids).
    std::set<int> member_track_ids;
  };

  /**
   * @brief A newly-added object as reported to sinks. Each track is an object, so `id` is the
   * track id, stable across merges since the older track survives.
   */
  struct AddedObject {
    //! Track id of the object.
    int id;
    //! The track id and the ids of all tracks it absorbed (Track::absorbed_track_ids).
    std::set<int> member_track_ids;
    //! First and last observation of the track.
    TimeStamp first_seen;
    TimeStamp last_seen;
    //! Accumulated bounding box of the track (Track::accumulated_bounding_box).
    BoundingBox bounding_box;
    //! Center of the bounding box.
    Point centroid;
    //! Semantic info of the track.
    std::optional<SemanticClusterInfo> semantics;
    //! Tracking-quality confidence (Track::confidence). Distinct from change_confidence -- this
    //! says how well-observed the object is, not how likely it is to be a genuine scene change.
    float confidence = 0.0f;
    //! Change-detection confidence (TrackAddedState::change_confidence); this, not `confidence`,
    //! is compared to added_probability_threshold.
    float change_confidence = 0.0f;
    //! TrackAddedState::num_frames_observed at the time this object was reported.
    int num_frames_observed = 0;
  };

  /**
   * @brief Debug status of one prior object considered for removal this frame (gate inputs,
   * measurement, filter state), including objects that fail the in-bounds gate.
   */
  struct RemovedCandidateStatus {
    spark_dsg::NodeId id;
    //! Whether the object has a mesh. Meshless objects use the centroid in-bounds test and can
    //! never be measured.
    bool has_geometry = false;
    //! Unique occupied map voxels of the object, and how many of them lie in allocated blocks.
    int num_voxels = 0;
    int num_voxels_in_bounds = 0;
    //! Whether the object passed the in-bounds gate (removal_min_in_bounds_fraction).
    bool in_bounds = false;
    //! Object voxels observed in the tracking layer (last_observed != 0), counted per voxel.
    //! Only computed for in-bounds objects.
    int num_voxels_observed = 0;
    //! Whether the object passed the observed gate (removal_min_observed_fraction).
    bool observed = false;
    //! Whether the object got a measurement this frame (passed both gates, >= 1 known sample).
    bool measured = false;
    //! Known and free samples this frame, in units of removal_score_weighting.
    int num_known = 0;
    int num_free = 0;
    //! Raw free ratio this frame (valid if measured).
    float raw_ratio = 0.0f;
    //! Filter state after this frame's update (frozen if not measured).
    float free_probability = 0.0f;
    int num_frames_observed = 0;
    //! Whether the object passed the removal gate this frame.
    bool removed = false;
    //! Object voxels (current map frame) by state. Only filled if collect_debug_voxels.
    std::vector<GlobalIndex> free_voxels;
    std::vector<GlobalIndex> occupied_voxels;
    std::vector<GlobalIndex> unknown_voxels;
  };

  /**
   * @brief Debug status of one track measured for newly-added detection this frame: its footprint
   * containment in the prior free space and its filter state.
   */
  struct AddedCandidateStatus {
    int track_id = -1;
    //! Whether the track passed the added gate this frame.
    bool added = false;
    //! Accumulated track bounding box in the current map frame.
    BoundingBox bounding_box;
    //! Raw containment this frame and the track's own filter state after the update.
    float raw_containment = 0.0f;
    float change_confidence = 0.0f;
    int num_frames_observed = 0;
    //! 2D footprint cells of the track, and how many of them lie in the prior free space.
    int num_cells = 0;
    int num_cells_in_free = 0;
    //! Footprint cell centers inside / outside the prior free space (at the prior place height /
    //! track centroid height). Only filled if collect_debug_voxels.
    std::vector<Point> in_free_cells;
    std::vector<Point> out_cells;
  };

  /**
   * @brief Per-frame debug status of the change detector for visualization. Separate from the
   * removed/added outputs, which only hold objects that passed the gates.
   */
  struct ChangeDetectionStatus {
    //! Map voxel size [m] of the voxel indices and cells below.
    float voxel_size = 0.0f;
    std::vector<RemovedCandidateStatus> removed_candidates;
    std::vector<AddedCandidateStatus> added_candidates;
  };

  using ActiveWindowCDSink = hydra::OutputSink<const DynamicSceneGraph::Ptr&,
                                               const std::vector<RemovedObject>&,
                                               const std::vector<AddedObject>&,
                                               const Eigen::Isometry3d&,
                                               const ChangeDetectionStatus&>;

  // Config.
  struct Config : hydra::VerbosityConfig {
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    //! Path to prior map to use for change detection.
    std::filesystem::path prior_map_path;

    //! EMA smoothing factor for the per-object free_probability filter (0 < alpha <= 1).
    //! Lower values give more smoothing (slower to react); higher values track raw ratios closely.
    //! Cumulative running average (alpha=1/n) is an easy drop-in if EMA proves too noisy.
    float removal_ema_alpha = 0.5f;

    //! Smoothed free_probability threshold above which an object is considered removed.
    float removal_probability_threshold = 0.5f;

    //! Minimum number of frames with valid measurements before an object can be declared removed.
    //! Guards against single-frame spikes on newly-entered objects.
    int removal_min_frames_observed = 3;

    //! Min fraction of a prior object's occupied voxels in allocated map blocks to be a removal
    //! candidate; others are not measured (EMA frozen). Meshless objects use the centroid.
    float removal_min_in_bounds_fraction = 0.5f;

    //! Min fraction of a prior object's occupied voxels observed in the tracking layer to get a
    //! measurement. Keep removal_min_in_bounds_fraction <= this as a cheap pre-filter.
    float removal_min_observed_fraction = 0.5f;

    //! Weight of each occupied map voxel in the removal free ratio: its number of mesh vertices
    //! (kVertices) or one (kVoxels).
    enum class RemovalScoreWeighting {
      kVertices,
      kVoxels
    } removal_score_weighting = RemovalScoreWeighting::kVertices;

    //! Fill the per-voxel lists of ChangeDetectionStatus for visualization. Scalar status fields
    //! are always filled.
    bool collect_debug_voxels = false;

    // Newly-added objects: each track is an object. Requires MaxIoUTracker accumulate_voxels;
    // without merge_tracks, fragments of one object are reported separately.

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

    //! Crop radius around robot (m) for mesh point selection.
    float icp_crop_radius = 10.0f;

    //! small_gicp threads.
    size_t icp_num_threads = 2;

    //! Voxel downsampling resolution (m).
    float icp_downsampling_resolution = 0.2f;

    //! Max ICP correspondence distance (m).
    float icp_max_correspondence_distance = 1.0f;

    //! Max small_gicp optimizer iterations. small_gicp's own default (20) combined with its tight
    //! default convergence tolerance (1mm translation / 0.1 deg rotation step size) rarely
    //! settles for noisy TSDF-mesh-to-mesh registration even with a good fit (high inlier count);
    //! raise this before loosening icp_max_correspondence_distance if converged=false persists
    //! with plenty of inliers.
    size_t icp_max_iterations = 50;

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
   * @brief Find all prior objects with at least removal_min_in_bounds_fraction of their occupied
   * voxels in allocated blocks. Objects without a mesh fall back to testing their centroid.
   * @param map The current volumetric map.
   * @param status If set, a RemovedCandidateStatus is appended for every object with any voxel in
   * bounds (or a meshless object passing the centroid test), including ones failing the gate.
   * @return Vector of node IDs for objects within map bounds.
   */
  std::vector<spark_dsg::NodeId> findPriorObjectsInMapBounds(
      const VolumetricMap& map, ChangeDetectionStatus* status = nullptr) const;

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
   * Samples are the object's cached map voxels, weighted per removal_score_weighting.
   * @param status If set, the measurement fields of the matching removed_candidates are filled.
   * @return Map from NodeId to raw free ratio [0, 1] for objects that had a valid measurement.
   */
  std::unordered_map<spark_dsg::NodeId, float> computeFreeRatios(
      const std::vector<spark_dsg::NodeId>& objects_id_in_bounds,
      const VolumetricMap& map,
      ChangeDetectionStatus* status = nullptr) const;

  /**
   * @brief Update the EMA filter state for each object that has a measurement this frame,
   * then return the set of objects whose smoothed free_probability passes the removal gate
   * (probability >= removal_probability_threshold && frames >= removal_min_frames_observed).
   * Objects not in measurements are left untouched (freeze-last policy). The first time an
   * object passes the gate, its RemovedObjectState::first_removed_ns is latched to `stamp`
   * (the current frame's sensor time) and never updated again.
   * @param measurements Per-object raw free ratio measurements for this frame.
   * @param stamp Sensor frame time (ns) of this call, used to latch first_removed_ns.
   */
  std::vector<RemovedObject> updateRemovedFilter(
      const std::unordered_map<spark_dsg::NodeId, float>& measurements, TimeStamp stamp) const;

  /**
   * @brief Compute the per-frame containment ratio for each eligible track (non-dynamic, with
   * confirmed accumulated voxels). Returns a map from track ID to raw containment ratio [0, 1].
   * Reuses getPriorFreeFootprint2D and getTrackFootprint2D.
   */
  std::unordered_map<int, float> computeContainmentRatios(
      const Tracks& tracks,
      const VolumetricMap& map,
      ChangeDetectionStatus* status = nullptr) const;

  /**
   * @brief Newly-added filter. Takes over the state of absorbed tracks, EMA-updates each measured
   * track and returns the tracks passing the added gate. Added tracks are latched; others not
   * measured for added_prune_after_frames frames are pruned.
   */
  std::vector<AddedObject> updateAddedFilter(const std::unordered_map<int, float>& measurements,
                                             const Tracks& tracks) const;

  //! EMA-update a track's change confidence with this frame's containment measurement.
  void updateTrackChangeConfidence(TrackAddedState& state, float containment) const;

  /**
   * @brief Cache each prior object's mesh vertices once, and re-quantize them into current-frame
   * map voxels and blocks whenever current_T_prior_ changes.
   */
  void updatePriorObjectGeometry(const VolumetricMap& map) const;

  // Returns 2D voxel indices (z forced to 0, in current frame) covering the prior
  // traversable (MESH_PLACES / TravNodeAttributes) footprint within map bounds.
  // If cell_heights is set, it receives the height (current frame) of each footprint cell's place.
  GlobalIndexSet getPriorFreeFootprint2D(
      const VolumetricMap& map, spatial_hash::IndexHashMap<float>* cell_heights = nullptr) const;

  // Returns 2D voxel indices (z forced to 0) of a track's confirmed accumulated voxels.
  GlobalIndexSet getTrackFootprint2D(const Track& track, float voxel_size) const;

  // Convert a 3-D point to a 2D voxel GlobalIndex (z component set to 0).
  static GlobalIndex to2DIndex(const Point& point, float voxel_size_inv);

 private:
  //! Prior map as a 3D scene graph.
  DynamicSceneGraph::Ptr prior_graph_;

  //! Transform from prior map frame to current map frame.
  //! current_point = current_T_prior_ * prior_point
  mutable Eigen::Isometry3d current_T_prior_ = Eigen::Isometry3d::Identity();

  //! A current-frame map voxel occupied by a prior object, with the number of its mesh vertices.
  struct ObjectVoxel {
    GlobalIndex index;
    int num_vertices = 0;
  };

  //! Occupied geometry of a prior object, used for the in-bounds test and the free ratio.
  struct PriorObjectGeometry {
    //! Mesh vertices in the prior map frame.
    std::vector<Eigen::Vector3d> vertices;
    //! Unique map voxels hit by the vertices, quantized in the current frame for the current
    //! current_T_prior_. Each voxel is exactly the map voxel a lookup of its vertices returns.
    std::vector<ObjectVoxel> voxels;
    //! Number of occupied voxels per map block, for the current current_T_prior_.
    spatial_hash::IndexHashMap<int> block_counts;
  };

  //! Per-object occupied geometry, keyed by prior DSG NodeId. Only objects with a mesh.
  mutable std::unordered_map<spark_dsg::NodeId, PriorObjectGeometry> prior_object_geometry_;

  //! Voxel size prior_object_geometry_ was built with (0 = not built yet).
  mutable float prior_geometry_voxel_size_ = 0.0f;

  //! Whether block_counts must be recomputed because current_T_prior_ changed.
  mutable bool prior_geometry_blocks_dirty_ = true;

  //! Per-object EMA filter state for removed-object detection. Keyed by prior DSG NodeId.
  //! Populated lazily on first observation; never erased (freeze-last for unobserved objects).
  mutable std::unordered_map<spark_dsg::NodeId, RemovedObjectState> removed_object_states_;

  //! Per-track state for newly-added-object detection, keyed by Track::id. Pruned for non-added
  //! tracks that go stale; added tracks are kept forever.
  mutable std::unordered_map<int, TrackAddedState> track_added_states_;

  //! Frame counter incremented once per call(), used for staleness-based pruning of added records.
  mutable int frame_index_ = 0;

  //! Plugin that provides the odom_T_prior transform.
  TransformationGetter::Ptr transformation_getter_;

  //! Sinks for the change detector output.
  ActiveWindowCDSink::List sinks_;
};

void declare_config(ActiveWindowChangeDetector::Config& config);

}  // namespace khronos
