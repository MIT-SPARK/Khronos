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
   * @brief A newly-added object as reported to sinks. Objects are persistent, identity-bearing
   * entities that accumulate evidence from associated Tracks (see AddedObjectState /
   * TrackAddedState) -- downstream consumers only ever see AddedObject, never Track. `id` comes
   * from a dedicated per-detector counter (next_object_id_), fully decoupled from any Track::id,
   * so it is stable for the object's whole lifetime regardless of which tracks associate to it
   * (barring `reassociate_every_frame`, which can move a track to a different object but never
   * changes an existing object's id).
   */
  struct AddedObject {
    //! Stable id for this object, from the detector's own next_object_id_ counter.
    int id;
    //! All track ids that have ever been associated to this object, for provenance/debugging.
    std::set<int> member_track_ids;
    //! Earliest first_seen over all associated tracks.
    TimeStamp first_seen;
    //! Latest last_seen over all associated tracks.
    TimeStamp last_seen;
    //! Incrementally-folded bounding box (see Config::bbox_merge_type: union or weighted average).
    BoundingBox bounding_box;
    //! Centroid of the folded bounding box.
    Point centroid;
    //! Fused semantic info (Track::updateSemantics-style aggregation over associated tracks).
    std::optional<SemanticClusterInfo> semantics;
    //! Tracking-quality confidence: max Track::confidence over ever-associated tracks. Distinct
    //! from change_confidence -- this says how well-observed the object is, not how likely it is
    //! to be a genuine scene change.
    float confidence = 0.0f;
    //! Change-detection confidence: max per-track free-space-containment EMA
    //! (TrackAddedState::change_confidence) over ever-associated tracks. This, not `confidence`,
    //! is what crosses added_probability_threshold to decide an object is newly-added.
    float change_confidence = 0.0f;
    //! AddedObjectState::num_frames_observed at the time this object was reported.
    int num_frames_observed = 0;
  };

  /**
   * @brief Per-track temporal state for the newly-added-object EMA filter, independent of any
   * object association. Keyed by Track::id in track_added_states_. A track's own EMA lives here
   * for its whole lifetime; association (below) only says which object it currently contributes
   * to, it does not replace or share this per-track state with any other track.
   */
  struct TrackAddedState {
    //! EMA-filtered probability that this track alone indicates a newly-added object (raw input:
    //! free-space containment of the track's footprint).
    double change_confidence = 0.0;
    //! Number of frames for which a valid containment measurement was available.
    int num_frames_observed = 0;
    //! Raw containment ratio from the most recent valid measurement (for logging/debugging).
    float last_containment = 0.0f;
    //! frame_index_ value at the last frame this track was measured (staleness pruning).
    int last_updated_frame = 0;
    //! Object this track currently contributes to, or -1 if not yet associated to any object.
    int object_id = -1;
  };

  /**
   * @brief Persistent, incrementally-updated state for one added object. Keyed by a dedicated
   * object id (next_object_id_) in added_object_states_ -- NOT by any Track::id. Updated by
   * folding in each newly-associated-or-updated track's info (see Config::bbox_merge_type for how
   * bounding boxes fold); never rebuilt from scratch. See AddedObject for the field-by-field
   * meaning of confidence vs. change_confidence -- this struct mirrors that split.
   */
  struct AddedObjectState {
    //! Incrementally-folded bounding box (see Config::bbox_merge_type).
    BoundingBox bounding_box;
    //! Earliest first_seen over all associated tracks.
    TimeStamp first_seen = 0;
    //! Latest last_seen over all associated tracks.
    TimeStamp last_seen = 0;
    //! Incrementally-fused semantic info (Track::updateSemantics-style aggregation).
    std::optional<SemanticClusterInfo> semantics;
    //! Tracking-quality confidence: max Track::confidence over ever-associated tracks. Also used
    //! as the blend weight for Config::BboxMergeType::kWeightedAverage.
    float confidence = 0.0f;
    //! Change-detection confidence: max over ever-associated tracks' own EMA
    //! (TrackAddedState::change_confidence). Drives the added-object gate, not `confidence`.
    double change_confidence = 0.0;
    //! Max TrackAddedState::num_frames_observed over ever-associated tracks.
    int num_frames_observed = 0;
    //! Latched true once the object passes the added gate; prevents pruning of confirmed objects.
    bool ever_added = false;
    //! frame_index_ value at the last frame any associated track was updated (staleness pruning).
    int last_updated_frame = 0;
    //! All track ids ever associated to this object, for provenance/debugging only -- no cached
    //! Track copies are kept; folding is incremental (see updateAddedFilter).
    std::set<int> member_track_ids;
  };

  using ActiveWindowCDSink = hydra::OutputSink<const DynamicSceneGraph::Ptr&,
                                               const std::vector<RemovedObject>&,
                                               const std::vector<AddedObject>&,
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

    //! Master switch for track-to-object association of added-object candidates (see
    //! "Association" in updateAddedFilter). Disable to fall back to one object per track id
    //! (no fusion of fragmented tracks at all).
    bool enable_added_object_merging = true;

    //! 3D bounding box IoU above which an unassociated track is considered a match for an
    //! existing object (geometric criterion, OR'd with merge_centroid_distance_threshold). <= 0
    //! disables this criterion (geometric match then relies solely on centroid distance).
    float merge_bbox_iou_threshold = 0.2f;

    //! Centroid distance [m] below which an unassociated track is considered a match for an
    //! existing object (geometric criterion, OR'd with merge_bbox_iou_threshold). <= 0 disables
    //! this criterion.
    float merge_centroid_distance_threshold = 0.5f;

    //! Minimum semantic feature cosine similarity required for a track to match an existing
    //! object. Passed to semanticsMatch; category_id must also match. This criterion is always
    //! required (AND'd with the geometric criteria above).
    float merge_min_semantic_cosine_sim = 0.7f;

    //! If true, an already-associated track re-runs candidate search against all current objects
    //! every frame and can switch association if a different object now scores better. Default
    //! false (sticky): a track keeps its first association forever. Note that switching does not
    //! retract the track's past contribution from its old object -- only future folds move.
    bool reassociate_every_frame = false;

    //! How an object's bounding box is updated when a track folds into it.
    enum class BboxMergeType {
      kUnion,           //! Smallest box enclosing both (spark_dsg::BoundingBox::merge). Can only grow.
      kWeightedAverage  //! Blend object's current box with the track's, weighted by confidence.
    } bbox_merge_type = BboxMergeType::kUnion;

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
   * Objects not in measurements are left untouched (freeze-last policy). The first time an
   * object passes the gate, its RemovedObjectState::first_removed_ns is latched to `stamp`
   * (the current frame's sensor time) and never updated again.
   * @param measurements Per-object raw free ratio measurements for this frame.
   * @param stamp Sensor frame time (ns) of this call, used to latch first_removed_ns.
   */
  std::vector<RemovedObject> updateRemovedFilter(
      const std::unordered_map<spark_dsg::NodeId, float>& measurements, TimeStamp stamp) const;

  /**
   * @brief Compute the per-frame containment ratio for each eligible track (non-dynamic,
   * non-empty footprint). Returns a map from track ID to raw containment ratio [0, 1].
   * Reuses getPriorFreeFootprint2D and getTrackFootprint2D.
   */
  std::unordered_map<int, float> computeContainmentRatios(const Tracks& tracks,
                                                          const VolumetricMap& map) const;

  /**
   * @brief For each track with a fresh containment measurement this frame: (1) EMA-update its
   * own TrackAddedState; (2) if already associated to an object, fold its info into that object
   * (or re-associate first, if reassociate_every_frame); if not yet associated, search existing
   * objects for the best match (findBestObjectMatch) and associate, or create a new object if
   * none matches. Then gates + prunes added_object_states_ on change_confidence (not confidence)
   * and returns the objects passing added_probability_threshold + added_min_frames_observed.
   * Tracks absent from measurements are frozen (their state and their object's are left as-is).
   */
  std::vector<AddedObject> updateAddedFilter(const std::unordered_map<int, float>& measurements,
                                             const Tracks& tracks) const;

  /**
   * @brief Search added_object_states_ for the best match for a candidate track/object footprint
   * (using bbox.world_P_center as the centroid), with the same criteria as before (semantics
   * required via semanticsMatch, AND bbox IoU >= merge_bbox_iou_threshold OR centroid distance <
   * merge_centroid_distance_threshold). Returns the highest-scoring matching object id (ties
   * broken by smaller centroid distance), or -1 if none passes. `exclude_id` (if >= 0) skips that
   * object id (used by reassociate_every_frame to also consider switching away from the track's
   * current object).
   */
  int findBestObjectMatch(const BoundingBox& bbox,
                          const std::optional<SemanticClusterInfo>& semantics,
                          int exclude_id = -1) const;

  /**
   * @brief Fold one track's current info (bbox/semantics/confidence/change_confidence) into an
   * existing AddedObjectState in place: bbox via Config::bbox_merge_type, confidence/
   * change_confidence/num_frames_observed via max, semantics fused Track::updateSemantics-style,
   * member_track_ids/last_updated_frame updated. For a brand-new object (state freshly
   * default-constructed), this initializes it directly from the track with no blending.
   */
  void foldTrackIntoObject(AddedObjectState& state,
                           const Track& track,
                           const TrackAddedState& track_state) const;

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

  //! Per-track EMA filter state for newly-added-object detection, independent of any object.
  //! Keyed by Track::id. TrackAddedState::object_id says which object (if any) this track
  //! currently contributes to.
  mutable std::unordered_map<int, TrackAddedState> track_added_states_;

  //! Persistent, incrementally-updated object state. Keyed by a dedicated object id (see
  //! next_object_id_), never by a Track::id. Pruned for non-added objects that go stale;
  //! confirmed-added records are kept forever.
  mutable std::unordered_map<int, AddedObjectState> added_object_states_;

  //! Monotonic counter minting the next object id; never reused.
  mutable int next_object_id_ = 0;

  //! Frame counter incremented once per call(), used for staleness-based pruning of added records.
  mutable int frame_index_ = 0;

  //! Plugin that provides the odom_T_prior transform.
  TransformationGetter::Ptr transformation_getter_;

  //! Sinks for the change detector output.
  ActiveWindowCDSink::List sinks_;
};

void declare_config(ActiveWindowChangeDetector::Config& config);

}  // namespace khronos
