#include "khronos_ros/active_window/external_object_extractor.h"

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace khronos {

namespace {

static const auto registration =
    config::RegistrationWithConfig<ObjectExtractor,
                                   ExternalObjectExtractor,
                                   ExternalObjectExtractor::Config>("ExternalObjectExtractor");

const MeasurementCluster* findClusterForId(const FrameData& frame, int target_id) {
  const auto it = std::find_if(frame.semantic_clusters.begin(),
                               frame.semantic_clusters.end(),
                               [&](const auto& cluster) { return cluster.id == target_id; });
  if (it == frame.semantic_clusters.end()) {
    return nullptr;
  }

  return &(*it);
}

void fillMessage(sensor_msgs::msg::Image& msg,
                 const cv::Mat& mat,
                 const std::string& encoding,
                 const std::optional<cv::Rect>& rect = std::nullopt,
                 const std_msgs::msg::Header& header = std_msgs::msg::Header{}) {
  cv::Mat to_use;
  if (rect) {
    to_use = cv::Mat(mat, *rect);
  } else {
    to_use = mat;
  }

  cv_bridge::CvImage img(header, encoding, to_use);
  img.toImageMsg(msg);
}

cv::Rect getClusterROI(const Pixels& pixels) {
  bool first = true;
  Eigen::Vector2i min;
  Eigen::Vector2i max;
  for (const auto& [u, v] : pixels) {
    const Eigen::Vector2i p_curr(u, v);
    if (first) {
      min = p_curr;
      max = p_curr;
      first = false;
    } else {
      min = min.cwiseMin(p_curr);
      max = max.cwiseMax(p_curr);
    }
  }

  const Eigen::Vector2i dims = max - min + Eigen::Vector2i::Ones();
  const cv::Rect roi(min.x(), min.y(), dims.x(), dims.y());
  VLOG(1) << "Got ROI: min=[" << min.x() << ", " << min.y() << "], max=[" << dims.x() << ", "
          << dims.y() << "]";

  return roi;
}

void fillMaskFromCluster(const MeasurementCluster& cluster, cv::Mat& mask) {
  for (const auto& [u, v] : cluster.pixels) {
    mask.at<uint8_t>(u, v) = 255;
  }
}

void fillMaskFromInstance(const FrameData& frame,
                          const MeasurementCluster& cluster,
                          cv::Mat& mask) {
  const auto& instances = frame.input.instance_image;
  for (int v = 0; v < instances.rows; ++v) {
    for (int u = 0; u < instances.cols; ++u) {
      const int id_value = instances.at<InputData::InstanceType>(v, u);
      mask.at<uint8_t>(v, u) = id_value == cluster.id ? 255 : 0;
    }
  }
}

using SrvResponse = khronos_msgs::srv::ExtractImplicitObject::Response;

std::unique_ptr<KhronosObjectAttributes> makeAttributes(const FrameData& frame,
                                                        const MeasurementCluster& cluster,
                                                        const SrvResponse& rep) {
  Eigen::Isometry3d cam_T_nocs;
  tf2::convert(rep.pose, cam_T_nocs);
  Eigen::Quaterniond cam_q_nocs(cam_T_nocs.rotation());
  const Eigen::Isometry3d world_T_cam = frame.input.getSensorPose();
  const Eigen::Isometry3d world_T_nocs = world_T_cam * cam_T_nocs;

  auto attrs = std::make_unique<KhronosObjectAttributes>();
  attrs->registered = true;
  attrs->position = world_T_nocs.translation();
  attrs->world_R_object = world_T_nocs.rotation();
  attrs->last_update_time_ns = frame.input.timestamp_ns;
  attrs->is_active = false;
  attrs->metadata.set({{"scale", rep.scale}});
  attrs->bounding_box = spark_dsg::BoundingBox(Eigen::Vector3f::Constant(rep.scale),
                                               attrs->position.cast<float>().eval(),
                                               attrs->world_R_object.cast<float>());
  attrs->semantic_label = cluster.semantics->category_id;
  attrs->semantic_feature =
      Eigen::Map<const Eigen::VectorXf>(rep.shape_code.data(), rep.shape_code.size());
  return attrs;
}

}  // namespace

using namespace std::chrono_literals;
using hydra::Camera;

void declare_config(ExternalObjectExtractor::Config& config) {
  using namespace config;
  name("ExternalObjectExtractor::Config");
  field(config.min_object_allocation_confidence, "min_object_allocation_confidence");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.excluded_labels, "excluded_labels");
  field(config.associate_lidar_name, "associate_lidar_name");
  enum_field(config.depth_mode,
             "depth_mode",
             {{ExternalObjectExtractor::Config::DepthMode::CAMERA_ONLY, "camera_only"},
              {ExternalObjectExtractor::Config::DepthMode::REPLACE_CAMERA, "replace_camera"},
              {ExternalObjectExtractor::Config::DepthMode::LIDAR_ONLY, "lidar_only"}});
  field(config.mask_from_cluster, "mask_from_cluster");
  field(config.filter_projected_depth, "filter_projected_depth");
  field(config.filter_cluster_tolerance, "filter_cluster_tolerance");
  field(config.filter_prefer_near, "filter_prefer_near");
  field(config.filter_quality_ratio, "filter_quality_ratio");
}

ExternalObjectExtractor::ExternalObjectExtractor(Config& config)
    : config(config::checkValid(config)),
      nh_(ianvs::NodeHandle::this_node("~")),
      client_(nh_.create_client<InferenceSrv>("detect_object")) {}

auto ExternalObjectExtractor::extractObject(const Track& track, const FrameDataBuffer& buffer)
    -> KhronosObjectAttributes::Ptr {
  if (track.confidence <= config.min_object_allocation_confidence) {
    return nullptr;
  }

  const auto best = getBestCluster(track, buffer);
  if (!best.frame) {
    return nullptr;
  }

  const auto base_ptr = &(best.frame->input.getSensor());
  const auto cam_ptr = dynamic_cast<const Camera*>(base_ptr);
  if (!cam_ptr) {
    LOG(ERROR) << "Camera sensor required!";
    return nullptr;
  }

  const auto cols = best.frame->input.color_image.cols;
  const auto rows = best.frame->input.color_image.rows;
  cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
  if (config.mask_from_cluster) {
    fillMaskFromCluster(*best.cluster, mask);
  } else {
    fillMaskFromInstance(*best.frame, *best.cluster, mask);
  }

  std::vector<const FrameData*> lidar_frames;
  if (!config.associate_lidar_name.empty() && config.depth_mode != Config::DepthMode::CAMERA_ONLY) {
    for (const auto& obs : track.observations) {
      auto lf = buffer.getData(obs.stamp, config.associate_lidar_name);
      if (lf) {
        lidar_frames.push_back(lf);
      }
    }
  }

  cv::Mat depth_to_send;
  switch (config.depth_mode) {
    case Config::DepthMode::LIDAR_ONLY:
      depth_to_send = createLidarOnlyDepthImage(*best.frame, lidar_frames, *cam_ptr);
      break;
    case Config::DepthMode::REPLACE_CAMERA:
      depth_to_send = replaceLidarDepthImage(*best.frame, lidar_frames, *cam_ptr);
      break;
    case Config::DepthMode::CAMERA_ONLY:
    default:
      depth_to_send = best.frame->input.depth_image;
      break;
  }

  if (config.filter_projected_depth) {
    filterDepthByCluster(depth_to_send, mask, *cam_ptr);
  }

  auto req = std::make_shared<InferenceSrv::Request>();
  fillMessage(req->rgb, best.frame->input.color_image, "rgb8");
  fillMessage(req->depth, depth_to_send, "32FC1");
  fillMessage(req->mask, mask, "8UC1");

  const auto& cam_config = cam_ptr->getConfig();
  req->intrinsics.k[0] = cam_config.fx;
  req->intrinsics.k[4] = cam_config.fy;
  req->intrinsics.k[2] = cam_config.cx;
  req->intrinsics.k[5] = cam_config.cy;
  req->volume = best.volume;
  req->semantic_id = best.cluster->semantics->category_id;

  const auto rep = ianvs::call_service(*client_, req);
  if (!rep) {
    LOG(ERROR) << "External service call failed!";
    return nullptr;
  }

  if (!rep->valid) {
    return nullptr;
  }

  VLOG(5) << "Got response features of " << rep->shape_code.size() << " elements";
  return makeAttributes(*best.frame, *best.cluster, *rep);
}

auto ExternalObjectExtractor::getBestCluster(const Track& track, const FrameDataBuffer& buffer)
    const -> InstanceResult {
  InstanceResult best;
  for (const auto& obs : track.observations) {
    const auto frame = buffer.getData(obs.stamp);
    if (!frame) {
      continue;
    }

    const auto sensor_name = frame->input.getSensor().name;
    // TODO(nathan) add config field
    if (sensor_name != "rgbd") {
      continue;
    }

    const auto cluster = findClusterForId(*frame, obs.semantic_cluster_id);
    if (!cluster) {
      continue;
    }

    const auto curr_volume = cluster->bounding_box.volume();
    if (curr_volume > best.volume) {
      best.volume = curr_volume;
      best.cluster_id = obs.semantic_cluster_id;
      best.frame = frame.get();
      best.cluster = cluster;
    }
  }

  if (!best.frame) {
    LOG(WARNING) << "Invalid track " << track.id << ": no observation in buffer!";
    return {};
  }

  if (best.cluster->pixels.size() < config.min_cluster_size) {
    LOG(ERROR) << "Too few pixels in best cluster: " << best.cluster->pixels.size() << " < "
               << config.min_cluster_size;
    return {};
  }

  if (!best.cluster->semantics) {
    LOG(ERROR) << "Semantics required for mesh extraction!";
    return {};
  }

  if (config.excluded_labels.count(best.cluster->semantics->category_id)) {
    VLOG(1) << "Skipping excluded label for track " << track.id;
    return {};
  }

  return best;
}

cv::Mat ExternalObjectExtractor::createLidarOnlyDepthImage(const FrameData& cam_frame,
                                                           const Frames& lidar_frames,
                                                           const Camera& camera) const {
  if (lidar_frames.empty()) {
    return cam_frame.input.depth_image;
  }

  const auto& depth_ref = cam_frame.input.depth_image;
  cv::Mat base_depth = cv::Mat::zeros(depth_ref.rows, depth_ref.cols, CV_32FC1);
  return projectLidarPoints(cam_frame, lidar_frames, camera, base_depth);
}

cv::Mat ExternalObjectExtractor::replaceLidarDepthImage(const FrameData& cam_frame,
                                                        const Frames& lidar_frames,
                                                        const Camera& camera) const {
  if (lidar_frames.empty()) {
    return cam_frame.input.depth_image;
  }

  cv::Mat base_depth = cam_frame.input.depth_image.clone();
  return projectLidarPoints(cam_frame, lidar_frames, camera, base_depth);
}

cv::Mat ExternalObjectExtractor::projectLidarPoints(const FrameData& cam_frame,
                                                    const Frames& lidar_frames,
                                                    const Camera& camera,
                                                    cv::Mat base_depth) const {
  const auto world_T_cam = cam_frame.input.getSensorPose();
  const Eigen::Isometry3f cam_T_world = world_T_cam.inverse().cast<float>();
  for (const auto& lidar_frame : lidar_frames) {
    if (!lidar_frame) {
      continue;
    }

    const auto points_in_world = lidar_frame->input.points_in_world_frame;
    Eigen::Isometry3f input_to_cam;
    if (points_in_world) {
      input_to_cam = cam_T_world;
    } else {
      const auto world_T_lidar = lidar_frame->input.getSensorPose();
      input_to_cam = (cam_T_world * world_T_lidar.cast<float>());
    }

    const cv::Mat& vertex_map = lidar_frame->input.vertex_map;
    for (int row = 0; row < vertex_map.rows; ++row) {
      for (int col = 0; col < vertex_map.cols; ++col) {
        const auto& p_raw = vertex_map.at<cv::Vec3f>(row, col);
        Eigen::Vector3f p_input(p_raw[0], p_raw[1], p_raw[2]);

        const float norm = p_input.norm();
        if (norm < 0.01f || !std::isfinite(norm)) {
          continue;
        }

        Eigen::Vector3f p_cam = input_to_cam * p_input;
        if (p_cam.z() <= 0.0f) {
          continue;
        }

        int u, v;
        if (!camera.projectPointToImagePlane(p_cam, u, v)) {
          continue;
        }

        if (u < 0 || u >= base_depth.cols || v < 0 || v >= base_depth.rows) {
          continue;
        }

        const float lidar_depth = p_cam.z();
        auto& current = base_depth.at<float>(v, u);
        if (current <= 0.0f || lidar_depth < current) {
          current = lidar_depth;
        }
      }
    }
  }

  return base_depth;
}

void ExternalObjectExtractor::filterDepthByCluster(cv::Mat& depth,
                                                   const cv::Mat& mask,
                                                   const Camera& camera) const {
  const auto& K = camera.getConfig();  // provides fx, fy, cx, cy

  // --- 1. Back-project masked pixels to 3D ---
  struct MaskedPoint {
    float x, y, z;
    int row, col;
  };
  std::vector<MaskedPoint> pts;
  pts.reserve(512);
  for (int r = 0; r < depth.rows; ++r) {
    for (int c = 0; c < depth.cols; ++c) {
      if (mask.at<uint8_t>(r, c) == 0) continue;
      const float d = depth.at<float>(r, c);
      if (d <= 0.f) continue;
      pts.push_back({(c - K.cx) * d / K.fx, (r - K.cy) * d / K.fy, d, r, c});
    }
  }

  if (pts.empty()) {
    return;
  }

  // --- 2. DBSCAN on 3D points ---
  const float eps2 = config.filter_cluster_tolerance * config.filter_cluster_tolerance;
  const int n = static_cast<int>(pts.size());
  std::vector<int> labels(n, -1);  // -1 = unvisited, 0 = noise, >0 = cluster id
  int num_clusters = 0;

  auto dist2 = [&](int i, int j) -> float {
    const float dx = pts[i].x - pts[j].x;
    const float dy = pts[i].y - pts[j].y;
    const float dz = pts[i].z - pts[j].z;
    return dx * dx + dy * dy + dz * dz;
  };

  for (int i = 0; i < n; ++i) {
    if (labels[i] != -1) continue;
    std::vector<int> neighbours;
    for (int j = 0; j < n; ++j) {
      if (dist2(i, j) <= eps2) neighbours.push_back(j);
    }

    if (static_cast<int>(neighbours.size()) < config.min_cluster_size) {
      labels[i] = 0;  // noise
      continue;
    }

    ++num_clusters;
    labels[i] = num_clusters;
    for (int ni = 0; ni < static_cast<int>(neighbours.size()); ++ni) {
      const int q = neighbours[ni];
      if (labels[q] == 0) labels[q] = num_clusters;  // noise promoted to border
      if (labels[q] != -1) continue;
      labels[q] = num_clusters;
      std::vector<int> q_nbrs;
      for (int j = 0; j < n; ++j) {
        if (dist2(q, j) <= eps2) q_nbrs.push_back(j);
      }

      if (static_cast<int>(q_nbrs.size()) >= config.min_cluster_size) {
        for (const int idx : q_nbrs) neighbours.push_back(idx);
      }
    }
  }

  if (num_clusters == 0) {
    CLOG(2) << "filterDepthByCluster: no valid clusters found, skipping.";
    return;
  }

  // --- 3. Select target cluster ---
  std::vector<double> sum_z(num_clusters + 1, 0.0);
  std::vector<int> cnt(num_clusters + 1, 0);
  for (int i = 0; i < n; ++i) {
    if (labels[i] > 0) {
      sum_z[labels[i]] += pts[i].z;
      ++cnt[labels[i]];
    }
  }

  // Largest cluster size (for quality thresholding)
  int max_cnt = 0;
  for (int k = 1; k <= num_clusters; ++k) {
    if (cnt[k] > max_cnt) max_cnt = cnt[k];
  }

  int target = -1;
  if (!config.filter_prefer_near) {
    // Pick the largest cluster — filtered-out points become outliers relative to it
    for (int k = 1; k <= num_clusters; ++k) {
      if (target == -1 || cnt[k] > cnt[target]) target = k;
    }
  } else {
    // Quality-filter: only consider clusters with cnt >= ratio * max_cnt, then pick nearest
    const int quality_threshold =
        static_cast<int>(config.filter_quality_ratio * static_cast<float>(max_cnt));
    float best_z = std::numeric_limits<float>::max();
    for (int k = 1; k <= num_clusters; ++k) {
      if (cnt[k] < quality_threshold) continue;
      const float mean_z = static_cast<float>(sum_z[k] / cnt[k]);
      if (mean_z < best_z) {
        best_z = mean_z;
        target = k;
      }
    }
    if (target == -1) {
      // Fallback: no cluster passed quality threshold — pick nearest unconditionally
      float best_z_fb = std::numeric_limits<float>::max();
      for (int k = 1; k <= num_clusters; ++k) {
        if (cnt[k] == 0) continue;
        const float mean_z = static_cast<float>(sum_z[k] / cnt[k]);
        if (mean_z < best_z_fb) {
          best_z_fb = mean_z;
          target = k;
        }
      }
    }
  }

  for (int i = 0; i < n; ++i) {
    if (labels[i] != target) {
      depth.at<float>(pts[i].row, pts[i].col) = 0.f;
    }
  }
}

}  // namespace khronos
