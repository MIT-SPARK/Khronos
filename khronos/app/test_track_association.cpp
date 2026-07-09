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

/**
 * Step 2 of the tracker-debugging pipeline (see tracker_debug.md): replays one track's saved
 * observations through the live MaxIoUTracker against another already-loaded track, to observe
 * directly whether/why the tracker associates them or spawns a new track. See
 * khronos/active_window/data/frame_data.h (FrameData::save/load) for the on-disk format this
 * reads -- Step 3 formalized that as a full FrameData serialization (color/depth/pose +
 * object_image/dynamic_image + clusters.json with real per-cluster semantics), so cluster
 * semantics used here are the actual per-frame values, not an aggregated-Track approximation.
 *
 * Inputs are `tracks/track_<id>/` directories written by ActiveWindowTrackSaver (see
 * track_saver_sink.cpp), each containing track.json plus symlinks back to the run's shared
 * `observations/<stamp>/` folders (color/depth/pose/segmentation, one per real frame).
 *
 * -------------------------------------------------------------------------------------------
 * USAGE
 * -------------------------------------------------------------------------------------------
 *
 * Binary: install/khronos/bin/test_track_association (after `colcon build --packages-select
 * khronos`). Run with --help to see all flags.
 *
 * IMPORTANT: pass the actual tracker config the run used (base_params/hydra.yaml or an
 * experiment override, e.g. dcist_launch_system/config/default_awcd/hydra.yaml's
 * `active_window.tracker` block), not the app's built-in MaxIoUTracker::Config defaults -- they
 * can differ (e.g. this codebase's default_awcd config uses min_semantic_iou: 0.25, not the
 * Config default of 0.5) and change whether a given IoU passes or fails.
 *
 * Mode 1 -- compare two different saved tracks (e.g. "did track_0 and track_3 fail to merge,
 * and why?"):
 *
 *   test_track_association \
 *     --existing-track-dir      <run_dir>/tracks/track_0 \
 *     --observation-track-dir   <run_dir>/tracks/track_3 \
 *     --bbox-type aabb --min-semantic-iou 0.25 --min-cross-iou 0.1 --min-num-observations 10 \
 *     --verbosity 6
 *
 *   - Replays every observation in --observation-track-dir (in chronological order) against the
 *     track loaded from --existing-track-dir. The run directory (containing camera_intrinsics.json
 *     and observations/) is derived automatically from each track dir's grandparent.
 *   - The existing track's last_points/last_bounding_box are auto-derived from its own historical
 *     observation immediately preceding --observation-track-dir's earliest observation (rather
 *     than its FINAL saved snapshot, which -- since Track only ever persists one, see Track::save
 *     -- may be far in time/viewpoint from the frames under test and would show artificially low
 *     IoU). No manual stamp-hunting needed. Pass --existing-track-stamp explicitly to override
 *     this auto-derivation with a different historical decision point.
 *   - --verbosity 6 surfaces MaxIoUTracker's own CLOG(6) accept/reject lines (low IoU vs.
 *     semantic mismatch, etc.) so you can see exactly why a cluster was or wasn't associated.
 *   - Per-frame RESULT lines report NEW TRACK CREATED / ASSOCIATED / dropped silently; a
 *     WARNING is printed if a newly-created track's id collides with the existing track's id (a
 *     fresh MaxIoUTracker's internal id counter starts at 0, same as many loaded tracks -- the
 *     harness tracks the "existing" track by its stable vector position, index 0, not by
 *     Track::id).
 *
 * Mode 2 -- self-consistency sanity check ("does the harness reproduce a track as ONE
 * continuous track when replayed from scratch, matching what production actually did?"):
 *
 *   test_track_association \
 *     --from-scratch \
 *     --observation-track-dir <run_dir>/tracks/track_0 \
 *     --bbox-type aabb --min-semantic-iou 0.25 --min-cross-iou 0.1 --min-num-observations 10
 *
 *   - Omit --existing-track-dir; starts with an empty track list and replays
 *     --observation-track-dir's own observation history in order, letting the tracker create
 *     and grow the track exactly as production's incremental processInput() calls would.
 *   - Ends with a summary: "PASS: stayed as ONE continuous track" if it never split, or "FAIL:
 *     fragmented into N tracks" with each fragment's id/observation count/time range if it did.
 *   - Useful to validate the harness itself isn't the source of a reported fragmentation before
 *     trusting a Mode 1 result.
 *
 * Both modes accept --save-reprojection-dir <dir> to additionally save, per replayed frame,
 * reprojection_<stamp>.png -- the candidate track's (tracks[0]) reprojected last_points in blue
 * overlaid with the frame's real detection cluster in green (only meaningful for the default
 * --track-by pixels; unset by default, so no files are written unless requested).
 */

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numeric>

#include <CLI/CLI.hpp>
#include <hydra/input/camera.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/data/track.h"
#include "khronos/active_window/tracking/max_iou_tracker.h"

using namespace khronos;

namespace {

std::vector<TimeStamp> sortedObservationStamps(const Track& track) {
  std::vector<TimeStamp> stamps;
  stamps.reserve(track.observations.size());
  for (const auto& obs : track.observations) {
    stamps.push_back(obs.stamp);
  }
  std::sort(stamps.begin(), stamps.end());
  return stamps;
}

// track_dir is always <run_dir>/tracks/track_<id> (see ActiveWindowTrackSaver); the run root
// (containing camera_intrinsics.json and observations/) is its grandparent.
std::string runDirFromTrackDir(const std::string& track_dir) {
  return std::filesystem::path(track_dir).parent_path().parent_path().string();
}

FrameData::Ptr loadObservation(const std::string& run_dir,
                               const std::shared_ptr<hydra::Camera>& camera,
                               TimeStamp stamp) {
  return FrameData::load(run_dir + "/observations/" + std::to_string(stamp), camera, stamp);
}

// Finds the (id, is_dynamic) of the cluster `track` was associated with at the given stamp, per
// its own saved observations[] list.
std::optional<std::pair<int, bool>> clusterIdAtStamp(const Track& track, TimeStamp stamp) {
  for (const auto& obs : track.observations) {
    if (obs.stamp != stamp) {
      continue;
    }
    if (obs.semantic_cluster_id >= 0) {
      return std::make_pair(obs.semantic_cluster_id, false);
    }
    if (obs.dynamic_cluster_id >= 0) {
      return std::make_pair(obs.dynamic_cluster_id, true);
    }
    return std::nullopt;
  }
  return std::nullopt;
}

// Finds the latest of `track`'s own observation stamps that is <= `before_stamp`. Used to
// auto-derive --existing-track-stamp: when comparing whether `observation_track` (e.g. track_3)
// should have associated into `track` (e.g. track_0), the relevant historical decision point is
// track_0's own state at its last observation before track_3 first appeared.
std::optional<TimeStamp> latestStampBefore(const Track& track, TimeStamp before_stamp) {
  std::optional<TimeStamp> best;
  for (const auto& obs : track.observations) {
    if (obs.stamp <= before_stamp && (!best || obs.stamp > *best)) {
      best = obs.stamp;
    }
  }
  return best;
}

// Removes every cluster from `frame_data` except the one with the given (id, is_dynamic) --
// preserves this harness's "does this one specific detection associate" semantics now that
// FrameData::load reconstructs the full multi-object frame. (Full multi-cluster competitive
// replay across the whole frame is Step 4's job, not this harness's.)
void keepOnlyCluster(FrameData* frame_data, int id, bool is_dynamic) {
  auto& keep = is_dynamic ? frame_data->dynamic_clusters : frame_data->semantic_clusters;
  auto& clear = is_dynamic ? frame_data->semantic_clusters : frame_data->dynamic_clusters;
  clear.clear();
  keep.erase(std::remove_if(keep.begin(),
                            keep.end(),
                            [id](const MeasurementCluster& c) { return c.id != id; }),
            keep.end());
}

// Track only ever persists its FINAL last_points/last_bounding_box snapshot (see
// Track::save/load), not a per-observation history. Replaying against that final snapshot
// reprojects points from whatever the track's very last observation was -- if that is far in time
// (and thus viewpoint) from the frame under test, computeIoUPixels will show low IoU purely from
// that staleness, regardless of whether the two detections are the same physical object. This
// rebuilds `track.last_points`/`last_bounding_box`/`semantics` as they actually were at a specific
// earlier on-disk observation of `track_dir` (using the REAL per-frame semantics from
// clusters.json, not the track's aggregated approximation), so replay can test the tracker's real
// decision point instead of an artifact of final-state persistence.
bool overrideWithHistoricalObservation(const std::string& track_dir, TimeStamp stamp, Track* track) {
  const std::string run_dir = runDirFromTrackDir(track_dir);
  const auto camera = loadCameraIntrinsics(run_dir);
  if (!camera) {
    return false;
  }

  const auto cluster_ref = clusterIdAtStamp(*track, stamp);
  if (!cluster_ref) {
    std::cerr << "Track has no observation at stamp " << stamp << " in " << track_dir << "\n";
    return false;
  }
  const auto [cluster_id, is_dynamic] = *cluster_ref;

  const auto frame_data = loadObservation(run_dir, camera, stamp);
  if (!frame_data) {
    std::cerr << "Failed to reconstruct historical observation at stamp " << stamp << " in "
             << track_dir << "\n";
    return false;
  }

  const auto& clusters = is_dynamic ? frame_data->dynamic_clusters : frame_data->semantic_clusters;
  const auto it = std::find_if(clusters.begin(), clusters.end(), [cluster_id](const auto& c) {
    return c.id == cluster_id;
  });
  if (it == clusters.end()) {
    std::cerr << "Cluster id " << cluster_id << " not found in observation at stamp " << stamp
             << "\n";
    return false;
  }

  Points last_points;
  last_points.reserve(it->pixels.size());
  for (const Pixel& pixel : it->pixels) {
    const auto& point = frame_data->input.vertex_map.at<hydra::InputData::VertexType>(pixel.v, pixel.u);
    last_points.emplace_back(point[0], point[1], point[2]);
  }

  track->last_points = std::move(last_points);
  track->last_bounding_box = it->bounding_box;
  track->semantics = it->semantics;  // real per-frame semantics, not the aggregated approximation.
  track->last_seen = stamp;
  return true;
}

// Saves a dual-color comparison image: cluster's real pixels in green, the track's reprojected
// points (see MaxIoUTracker::reprojectPoints) in blue -- drawn second so overlap reads as blue.
// Mirrors ActiveWindowTrackSaver::saveOverlay's blend style (0.6/0.4 addWeighted, applied twice).
void saveReprojectionOverlay(const std::string& dir,
                             TimeStamp stamp,
                             const FrameData& frame_data,
                             const MeasurementCluster& cluster,
                             const std::set<Pixel>& reprojected_pixels) {
  if (frame_data.input.color_image.empty()) {
    return;
  }
  std::filesystem::create_directories(dir);

  cv::Mat bgr_image;
  cv::cvtColor(frame_data.input.color_image, bgr_image, cv::COLOR_RGB2BGR);

  cv::Mat detection_mask = cv::Mat::zeros(bgr_image.size(), CV_8UC1);
  for (const Pixel& pixel : cluster.pixels) {
    if (pixel.isInImage(detection_mask)) {
      detection_mask.at<uint8_t>(pixel.v, pixel.u) = 255;
    }
  }
  cv::Mat reprojected_mask = cv::Mat::zeros(bgr_image.size(), CV_8UC1);
  for (const Pixel& pixel : reprojected_pixels) {
    if (pixel.isInImage(reprojected_mask)) {
      reprojected_mask.at<uint8_t>(pixel.v, pixel.u) = 255;
    }
  }

  cv::Mat overlay = bgr_image.clone();
  overlay.setTo(cv::Scalar(0, 255, 0), detection_mask);   // Green in BGR: real detection.
  overlay.setTo(cv::Scalar(255, 0, 0), reprojected_mask);  // Blue in BGR: reprojected track.
  cv::Mat blended;
  cv::addWeighted(bgr_image, 0.6, overlay, 0.4, 0, blended);
  cv::imwrite(dir + "/reprojection_" + std::to_string(stamp) + ".png", blended);
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 0;

  CLI::App app("Replay an observation track through MaxIoUTracker against an existing track");

  std::string existing_track_dir;
  std::string observation_track_dir;
  int verbosity = 6;
  float min_semantic_iou = 0.25f;
  float min_cosine_sim = 0.0f;
  float min_cross_iou = 0.1f;
  int min_num_observations = 10;
  std::string bbox_type = "aabb";
  long long existing_track_stamp = -1;
  std::string save_reprojection_dir;

  bool from_scratch = false;

  app.add_option("--existing-track-dir", existing_track_dir,
                 "Directory (e.g. .../track_0) of the track to treat as already-tracked. Omit "
                 "with --from-scratch to instead replay --observation-track-dir's own history "
                 "starting from an empty track list (self-consistency sanity check).")
      ->check(CLI::ExistingDirectory);
  app.add_flag("--from-scratch", from_scratch,
              "Ignore --existing-track-dir; start with no tracks and replay "
              "--observation-track-dir's own observation history from scratch, to check whether "
              "the harness reproduces it as a single continuous track (sanity check that the "
              "harness matches production behavior, not a test of fragmentation against a "
              "*different* track).");
  app.add_option("--observation-track-dir", observation_track_dir,
                 "Directory (e.g. .../track_3) whose observations are replayed as new detections")
      ->required()
      ->check(CLI::ExistingDirectory);
  app.add_option("--verbosity", verbosity,
                "Tracker verbosity; >=6 prints per-cluster accept/reject reasons")
      ->default_val(6);
  app.add_option("--min-semantic-iou", min_semantic_iou, "MaxIoUTracker::Config::min_semantic_iou")
      ->default_val(0.5f);
  app.add_option("--min-cosine-sim", min_cosine_sim, "MaxIoUTracker::Config::min_cosine_sim")
      ->default_val(0.0f);
  app.add_option("--min-cross-iou", min_cross_iou, "MaxIoUTracker::Config::min_cross_iou")
      ->default_val(0.5f);
  app.add_option(
      "--min-num-observations", min_num_observations, "MaxIoUTracker::Config::min_num_observations")
      ->default_val(20);
  app.add_option("--bbox-type", bbox_type, "MaxIoUTracker::Config::bbox_type (aabb|raabb)")
      ->default_val("aabb");
  app.add_option("--existing-track-stamp", existing_track_stamp,
                "Optional: override the existing track's last_points/last_bounding_box using its "
                "OWN on-disk observation at this stamp (ns), instead of its final saved snapshot. "
                "If omitted, this is auto-derived as the existing track's latest observation stamp "
                "at or before --observation-track-dir's earliest observation stamp -- pass this "
                "flag explicitly only to override that choice.");
  app.add_option("--save-reprojection-dir", save_reprojection_dir,
                "Optional: directory to save per-frame reprojection comparison images "
                "(reprojection_<stamp>.png) -- the seed/candidate track (tracks[0])'s reprojected "
                "last_points in blue, overlaid with the real detection cluster in green. Only "
                "meaningful when --track-by is 'pixels' (the default) and a candidate track with "
                "non-empty last_points exists; unset by default (no files written). In "
                "--from-scratch mode with fragmentation, this only visualizes against tracks[0].");

  CLI11_PARSE(app, argc, argv);

  if (!from_scratch && existing_track_dir.empty()) {
    std::cerr << "Either --existing-track-dir or --from-scratch is required.\n";
    return EXIT_FAILURE;
  }

  const Track observation_track = Track::load(observation_track_dir + "/track.json");
  std::cout << "Loaded observation track id=" << observation_track.id
           << " (" << observation_track.observations.size() << " observations, "
           << observation_track.first_seen << " -> " << observation_track.last_seen << ")\n";

  Tracks tracks;
  if (!from_scratch) {
    Track existing_on_disk = Track::load(existing_track_dir + "/track.json");
    if (existing_track_stamp < 0) {
      const auto observation_stamps = sortedObservationStamps(observation_track);
      if (!observation_stamps.empty()) {
        const auto derived = latestStampBefore(existing_on_disk, observation_stamps.front());
        if (derived) {
          existing_track_stamp = static_cast<long long>(*derived);
          std::cout << "Auto-derived --existing-track-stamp " << existing_track_stamp
                   << " (existing track's latest observation at or before observation track's "
                      "earliest stamp " << observation_stamps.front() << "). Pass "
                      "--existing-track-stamp explicitly to override.\n";
        } else {
          std::cout << "No auto-derivation possible: existing track has no observation at or "
                      "before observation track's earliest stamp " << observation_stamps.front()
                   << "; using existing track's final saved snapshot instead.\n";
        }
      }
    }
    if (existing_track_stamp >= 0) {
      if (!overrideWithHistoricalObservation(existing_track_dir,
                                             static_cast<TimeStamp>(existing_track_stamp),
                                             &existing_on_disk)) {
        return EXIT_FAILURE;
      }
      std::cout << "Overrode existing track's last_points/last_bounding_box using its own "
                  "observation at stamp " << existing_track_stamp << "\n";
    }
    std::cout << "Loaded existing track id=" << existing_on_disk.id
             << " (" << existing_on_disk.observations.size() << " observations, "
             << existing_on_disk.first_seen << " -> " << existing_on_disk.last_seen << ")\n";
    tracks = {existing_on_disk};
  } else {
    std::cout << "Starting from scratch (no seed track); replaying "
              << observation_track.observations.size()
              << " of the observation track's own observations in order.\n";
  }

  MaxIoUTracker::Config config;
  config.verbosity = verbosity;
  config.min_semantic_iou = min_semantic_iou;
  config.min_cosine_sim = min_cosine_sim;
  config.min_cross_iou = min_cross_iou;
  config.min_num_observations = min_num_observations;
  config.bbox_type = bbox_type == "raabb" ? MaxIoUTracker::Config::BBoxType::kRAABB
                                          : MaxIoUTracker::Config::BBoxType::kAABB;
  MaxIoUTracker tracker(config);

  // NOTE: a freshly constructed MaxIoUTracker starts its internal id counter at 0
  // (current_track_id_), which can collide with a loaded existing track's on-disk id (also
  // frequently 0). So the "existing"/"seed" track (when present) is identified by its STABLE
  // VECTOR POSITION (index 0 -- addNewTrack only ever tracks.emplace_back()s, never inserts
  // before or reorders), not by Track::id, which cannot be trusted to stay unique against newly
  // minted tracks in this harness.
  const bool has_seed_track = !tracks.empty();

  const std::string observation_run_dir = runDirFromTrackDir(observation_track_dir);
  const auto observation_camera = loadCameraIntrinsics(observation_run_dir);
  if (!observation_camera) {
    return EXIT_FAILURE;
  }

  for (const TimeStamp stamp : sortedObservationStamps(observation_track)) {
    const auto cluster_ref = clusterIdAtStamp(observation_track, stamp);
    if (!cluster_ref) {
      std::cerr << "Observation track has no cluster id at stamp " << stamp << "; skipping.\n";
      continue;
    }
    const auto [cluster_id, is_dynamic] = *cluster_ref;

    const auto frame_data = loadObservation(observation_run_dir, observation_camera, stamp);
    if (!frame_data) {
      std::cerr << "Failed to reconstruct FrameData for stamp " << stamp << "; skipping.\n";
      continue;
    }
    // Only test whether THIS specific detection associates with the seed track -- clear every
    // other cluster in the frame so the tracker can't associate against unrelated objects.
    keepOnlyCluster(frame_data.get(), cluster_id, is_dynamic);

    if (!save_reprojection_dir.empty() &&
       config.track_by == MaxIoUTracker::Config::TrackBy::kPixels && !tracks.empty() &&
       !tracks[0].last_points.empty()) {
      const auto& kept_clusters = is_dynamic ? frame_data->dynamic_clusters : frame_data->semantic_clusters;
      if (!kept_clusters.empty()) {
        const auto reprojected_pixels = tracker.reprojectPoints(*frame_data, tracks[0].last_points);
        saveReprojectionOverlay(
            save_reprojection_dir, stamp, *frame_data, kept_clusters.front(), reprojected_pixels);
      }
    }

    const size_t tracks_before = tracks.size();
    const size_t seed_obs_before = has_seed_track ? tracks[0].observations.size() : 0;

    tracker.processInput(*frame_data, tracks);

    std::cout << "\n=== Replayed observation stamp " << stamp << " ===\n";
    if (tracks.size() > tracks_before) {
      // The very first frame in --from-scratch mode goes 0 -> 1: that is the initial track
      // being created, not a fragmentation event.
      const bool is_initial_creation = !has_seed_track && tracks_before == 0;
      std::cout << "RESULT: " << (is_initial_creation ? "INITIAL TRACK CREATED" : "NEW TRACK "
                                  "CREATED (fragmentation reproduced)")
               << ". tracks.size() " << tracks_before << " -> " << tracks.size() << "\n";
    } else if (has_seed_track) {
      const size_t seed_obs_after = tracks[0].observations.size();
      if (seed_obs_after > seed_obs_before) {
        std::cout << "RESULT: ASSOCIATED into seed track (observations " << seed_obs_before
                 << " -> " << seed_obs_after << ")\n";
      } else {
        std::cout << "RESULT: no track count change and no observation added -- detection was "
                    "dropped silently.\n";
      }
    } else {
      const size_t total_obs_after =
          std::accumulate(tracks.begin(), tracks.end(), size_t{0},
                          [](size_t sum, const Track& t) { return sum + t.observations.size(); });
      std::cout << "RESULT: associated into one of the " << tracks.size()
               << " existing track(s) (total observations now " << total_obs_after << ")\n";
    }
  }

  std::cout << "\n=== Summary ===\n";
  std::cout << "Total distinct tracks after replay: " << tracks.size() << "\n";
  if (tracks.size() == 1) {
    std::cout << "PASS: stayed as ONE continuous track across all replayed observations.\n";
  } else {
    std::cout << "FAIL: fragmented into " << tracks.size() << " tracks:\n";
    for (size_t i = 0; i < tracks.size(); ++i) {
      std::cout << "  [" << i << "] id=" << tracks[i].id << " observations="
               << tracks[i].observations.size() << " first_seen=" << tracks[i].first_seen
               << " last_seen=" << tracks[i].last_seen << "\n";
    }
  }

  return EXIT_SUCCESS;
}
