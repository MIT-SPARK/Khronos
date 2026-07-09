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
 * Step 4 of the tracker-debugging pipeline (see tracker_debug.md): replays an ENTIRE saved run
 * through a fresh MaxIoUTracker, frame by frame in chronological order, with the real multi-object
 * competition production has (every cluster in every frame, not the single-pair filtering
 * test_track_association.cpp does for Step 2). Lets different tracker configs (min_semantic_iou,
 * track_by, etc.) be evaluated end-to-end against the same recorded data.
 *
 * Also simulates active-window eviction the same way production does: after each frame's
 * association, every track's is_active is recomputed via the real hydra::VolumetricWindow
 * (SpatialWindowChecker/TemporalWindowChecker) inBounds() check, exactly mirroring
 * ActiveWindow::updateTrackingStatus (active_window.cpp). Evicted tracks are erased from the live
 * list (mirroring ActiveWindow::extractInactiveObjects) and their final state archived to disk --
 * once evicted, a track can never be re-associated again, matching production.
 *
 * KNOWN APPROXIMATION: pose.txt records getSensorPose() = real_world_T_body * real_body_T_sensor
 * from the original run. FrameData::load's reconstructed camera uses identity extrinsics, so
 * input.world_T_body is set to that recorded pose directly -- correct for the tracker's own
 * reprojection math, but when reused here as "the robot body pose" for the eviction-radius check,
 * it is off by the real camera's mounting offset (typically tens of cm). Negligible at the
 * deployed 14m threshold; not solved (would require saving the real extrinsics).
 *
 * -------------------------------------------------------------------------------------------
 * USAGE
 * -------------------------------------------------------------------------------------------
 *
 * Binary: install/khronos/bin/full_run_tracker_replay (after `colcon build --packages-select
 * khronos`). Run with --help to see all flags.
 *
 *   full_run_tracker_replay \
 *     --run-dir <run_dir> --output-dir <output_dir> \
 *     --bbox-type aabb --min-semantic-iou 0.25 --min-cross-iou 0.1 --min-num-observations 10 \
 *     --track-by pixels --map-window-type spatial --map-window-radius 14.0
 *
 * IMPORTANT: --track-by / --min-semantic-iou / etc. default to MaxIoUTracker::Config's in-code
 * defaults, NOT necessarily what a given run's config used (e.g. this codebase's default_awcd
 * config uses min_semantic_iou: 0.25, not the Config default of 0.5) -- pass the real deployed
 * values (base_params/hydra.yaml or an experiment override's active_window.tracker/map_window
 * blocks) to reproduce a run faithfully, or deliberately change them to test whether a different
 * config would have avoided a known fragmentation case.
 *
 * --pause-on-event blocks on stdin (press Enter to continue) only at frames where a track was
 * created or evicted -- the moments actually worth inspecting -- instead of every frame.
 *
 * --save-overlay (off by default) additionally writes overlay_<ts>.png (color + red mask blend)
 * into each track's output directory for every frame it is updated, for offline visual QA of
 * which detections got associated into which track. Off by default since it adds one image write
 * per track per frame, which adds up over a full run.
 *
 * Output layout:
 *   <output_dir>/tracks/track_<id>/track.json              # tracks still active when the run ended
 *   <output_dir>/tracks/track_<id>/overlay_<ts>.png         # with --save-overlay
 *   <output_dir>/archived_tracks/track_<id>/track.json       # tracks evicted during the run
 *   <output_dir>/archived_tracks/track_<id>/overlay_<ts>.png # with --save-overlay (moves with the
 *                                                            # track when it's evicted)
 * track.json is a plain Track::save() file -- diffable against the original run's real tracks/
 * folder, and directly loadable by test_track_association for further investigation of any new
 * fragmentation this replay surfaces.
 */

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <set>

#include <CLI/CLI.hpp>
#include <hydra/active_window/volumetric_window.h>
#include <hydra/input/camera.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/data/track.h"
#include "khronos/active_window/tracking/max_iou_tracker.h"

using namespace khronos;

namespace {

std::vector<TimeStamp> sortedObservationStamps(const std::string& run_dir) {
  std::vector<TimeStamp> stamps;
  for (const auto& entry : std::filesystem::directory_iterator(run_dir + "/observations")) {
    if (!entry.is_directory()) {
      continue;
    }
    try {
      stamps.push_back(static_cast<TimeStamp>(std::stoull(entry.path().filename().string())));
    } catch (const std::exception&) {
      std::cerr << "Skipping non-timestamp observations/ entry: " << entry.path() << "\n";
    }
  }
  std::sort(stamps.begin(), stamps.end());
  return stamps;
}

std::set<int> trackIds(const Tracks& tracks) {
  std::set<int> ids;
  for (const auto& track : tracks) {
    ids.insert(track.id);
  }
  return ids;
}

// Working directory for a track while it is still alive (whether it ends up still-active at run
// end or evicted partway through) -- also where --save-overlay images accumulate over the track's
// lifetime, so an evicted track's overlay history moves with it (see archiveTrack).
std::string trackWorkingDir(const std::string& output_dir, int track_id) {
  return output_dir + "/tracks/track_" + std::to_string(track_id);
}

void archiveTrack(const std::string& output_dir, const Track& track) {
  const std::string working_dir = trackWorkingDir(output_dir, track.id);
  const std::string archived_dir =
      output_dir + "/archived_tracks/track_" + std::to_string(track.id);
  std::filesystem::create_directories(output_dir + "/archived_tracks");
  if (std::filesystem::exists(working_dir)) {
    // Move (not copy) so any overlay_<ts>.png written during the track's life moves with it,
    // rather than being duplicated or left behind in tracks/.
    std::error_code ec;
    std::filesystem::rename(working_dir, archived_dir, ec);
    if (ec) {
      std::filesystem::create_directories(archived_dir);
    }
  } else {
    std::filesystem::create_directories(archived_dir);
  }
  track.save(archived_dir + "/track.json");
}

// Writes overlay_<ts>.png (color image with the track's current-frame mask blended in red) into
// the track's working directory, for offline visualization. Only called when --save-overlay is
// set; mirrors ActiveWindowTrackSaver::saveOverlay's blending logic.
void saveOverlay(const std::string& output_dir, const Track& track, const FrameData& frame_data) {
  if (track.observations.empty() || frame_data.input.color_image.empty()) {
    return;
  }
  const Observation& obs = track.observations.back();

  cv::Mat binary_mask;
  if (obs.semantic_cluster_id >= 0 && !frame_data.object_image.empty()) {
    binary_mask = (frame_data.object_image == obs.semantic_cluster_id);
  } else if (obs.dynamic_cluster_id >= 0 && !frame_data.dynamic_image.empty()) {
    binary_mask = (frame_data.dynamic_image == obs.dynamic_cluster_id);
  } else {
    return;
  }

  const std::string dir = trackWorkingDir(output_dir, track.id);
  std::filesystem::create_directories(dir);

  cv::Mat bgr_image;
  cv::cvtColor(frame_data.input.color_image, bgr_image, cv::COLOR_RGB2BGR);
  cv::Mat overlay = bgr_image.clone();
  overlay.setTo(cv::Scalar(0, 0, 255), binary_mask);  // Red in BGR.
  cv::Mat blended;
  cv::addWeighted(bgr_image, 0.6, overlay, 0.4, 0, blended);
  cv::imwrite(dir + "/overlay_" + std::to_string(obs.stamp) + ".png", blended);
}

void saveActiveTrack(const std::string& output_dir, const Track& track) {
  const std::string dir = trackWorkingDir(output_dir, track.id);
  std::filesystem::create_directories(dir);
  track.save(dir + "/track.json");
}

void pauseForEnter(const std::string& message) {
  std::cout << message << " (press Enter to continue)";
  std::cout.flush();
  std::cin.get();
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 0;

  CLI::App app("Replay an entire saved run through a fresh MaxIoUTracker for parameter tuning");

  std::string run_dir;
  std::string output_dir;
  int verbosity = 0;
  float min_semantic_iou = 0.5f;
  float min_cosine_sim = 0.0f;
  float min_cross_iou = 0.5f;
  int min_num_observations = 20;
  std::string bbox_type = "aabb";
  std::string track_by = "pixels";
  std::string map_window_type = "spatial";
  double map_window_radius = 14.0;
  double map_window_seconds = 3.0;
  bool pause_on_event = false;
  bool save_overlay = false;

  app.add_option("--run-dir", run_dir, "Run directory (has camera_intrinsics.json + observations/)")
      ->required()
      ->check(CLI::ExistingDirectory);
  app.add_option("--output-dir", output_dir, "Where to write tracks/ and archived_tracks/")
      ->required();
  app.add_option("--verbosity", verbosity,
                "Tracker verbosity; >=6 prints per-cluster accept/reject reasons")
      ->default_val(0);
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
  app.add_option("--track-by", track_by, "MaxIoUTracker::Config::track_by (pixels|voxels|bounding_box)")
      ->default_val("pixels");
  app.add_option("--map-window-type", map_window_type,
                "Active-window eviction policy (spatial|temporal), matches hydra's map_window config")
      ->default_val("spatial");
  app.add_option("--map-window-radius", map_window_radius,
                "SpatialWindowChecker::Config::max_radius_m [m]; used if --map-window-type=spatial")
      ->default_val(14.0);
  app.add_option("--map-window-seconds", map_window_seconds,
                "TemporalWindowChecker::Config::window_sec [s]; used if --map-window-type=temporal")
      ->default_val(3.0);
  app.add_flag("--pause-on-event", pause_on_event,
              "Block on stdin (press Enter) only at frames where a track was created or evicted, "
              "instead of every frame.");
  app.add_flag("--save-overlay", save_overlay,
              "Save overlay_<ts>.png (color + red mask blend) into each track's output directory "
              "for every frame it is updated, for offline visualization. Off by default -- adds "
              "one image write per track per frame, which can be significant over a full run.");

  CLI11_PARSE(app, argc, argv);

  const auto camera = loadCameraIntrinsics(run_dir);
  if (!camera) {
    return EXIT_FAILURE;
  }

  MaxIoUTracker::Config tracker_config;
  tracker_config.verbosity = verbosity;
  tracker_config.min_semantic_iou = min_semantic_iou;
  tracker_config.min_cosine_sim = min_cosine_sim;
  tracker_config.min_cross_iou = min_cross_iou;
  tracker_config.min_num_observations = min_num_observations;
  tracker_config.bbox_type = bbox_type == "raabb" ? MaxIoUTracker::Config::BBoxType::kRAABB
                                                  : MaxIoUTracker::Config::BBoxType::kAABB;
  if (track_by == "voxels") {
    tracker_config.track_by = MaxIoUTracker::Config::TrackBy::kVoxels;
  } else if (track_by == "bounding_box") {
    tracker_config.track_by = MaxIoUTracker::Config::TrackBy::kBouningBox;
  } else {
    tracker_config.track_by = MaxIoUTracker::Config::TrackBy::kPixels;
  }
  MaxIoUTracker tracker(tracker_config);

  std::unique_ptr<hydra::VolumetricWindow> map_window;
  if (map_window_type == "temporal") {
    hydra::TemporalWindowChecker::Config cfg;
    cfg.window_sec = map_window_seconds;
    map_window = std::make_unique<hydra::TemporalWindowChecker>(cfg);
  } else {
    hydra::SpatialWindowChecker::Config cfg;
    cfg.max_radius_m = map_window_radius;
    map_window = std::make_unique<hydra::SpatialWindowChecker>(cfg);
  }

  const auto stamps = sortedObservationStamps(run_dir);
  std::cout << "Replaying " << stamps.size() << " observations from " << run_dir << "\n";

  Tracks tracks;
  size_t total_tracks_created = 0;
  size_t total_tracks_archived = 0;

  for (const TimeStamp stamp : stamps) {
    const auto frame_data =
        FrameData::load(run_dir + "/observations/" + std::to_string(stamp), camera, stamp);
    if (!frame_data) {
      std::cerr << "Failed to reconstruct FrameData for stamp " << stamp << "; skipping.\n";
      continue;
    }

    const auto ids_before = trackIds(tracks);
    tracker.processInput(*frame_data, tracks);

    std::vector<int> new_ids;
    for (const auto& track : tracks) {
      if (!ids_before.count(track.id)) {
        new_ids.push_back(track.id);
      }
    }
    total_tracks_created += new_ids.size();

    if (save_overlay) {
      for (const auto& track : tracks) {
        if (track.last_seen == stamp) {
          saveOverlay(output_dir, track, *frame_data);
        }
      }
    }

    // Mirrors ActiveWindow::updateTrackingStatus: recompute is_active for every track using the
    // real eviction policy, keyed off this frame's robot pose.
    for (auto& track : tracks) {
      const Eigen::Vector3d track_pos = track.last_bounding_box.world_P_center.cast<double>();
      track.is_active = map_window->inBounds(
          stamp, frame_data->input.world_T_body, track.last_seen, track_pos);
    }

    // Mirrors ActiveWindow::extractInactiveObjects: erase and archive newly-inactive tracks.
    std::vector<int> evicted_ids;
    auto it = tracks.begin();
    while (it != tracks.end()) {
      if (it->is_active) {
        ++it;
        continue;
      }
      evicted_ids.push_back(it->id);
      archiveTrack(output_dir, *it);
      ++total_tracks_archived;
      it = tracks.erase(it);
    }

    if (pause_on_event && (!new_ids.empty() || !evicted_ids.empty())) {
      std::cout << "\n[stamp " << stamp << "] ";
      if (!new_ids.empty()) {
        std::cout << "NEW TRACK(S): ";
        for (int id : new_ids) {
          std::cout << id << " ";
        }
      }
      if (!evicted_ids.empty()) {
        std::cout << "EVICTED: ";
        for (int id : evicted_ids) {
          std::cout << id << " ";
        }
      }
      std::cout << "| tracks currently active: " << tracks.size();
      pauseForEnter("");
    }
  }

  for (const auto& track : tracks) {
    saveActiveTrack(output_dir, track);
  }

  std::cout << "\n=== Summary ===\n";
  std::cout << "Frames processed: " << stamps.size() << "\n";
  std::cout << "Total tracks created: " << total_tracks_created << "\n";
  std::cout << "Active at end: " << tracks.size() << "\n";
  std::cout << "Archived (evicted) during run: " << total_tracks_archived << "\n";
  std::cout << "Results written to: " << output_dir << " (tracks/ + archived_tracks/)\n";

  return EXIT_SUCCESS;
}
