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

using SrvResponse = khronos_msgs::srv::ExtractImplicitObject::Response;

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

struct ImagePoint {
  int u;
  int v;
  Eigen::Vector3f pos;
};

void fillMaskPoints(const hydra::Camera& camera,
                    const cv::Mat& mask,
                    const cv::Mat& depth,
                    std::vector<ImagePoint>& points) {
  for (int r = 0; r < depth.rows; ++r) {
    for (int c = 0; c < depth.cols; ++c) {
      if (mask.at<uint8_t>(r, c) == 0) {
        continue;
      }

      const auto d = depth.at<float>(r, c);
      if (!std::isfinite(d) || d <= 0.f) {
        continue;
      }

      points.push_back({c, r, camera.unprojectPixel(c, r, d)});
    }
  }
}

size_t runDBSCAN(const std::vector<ImagePoint>& points,
                 std::vector<int>& labels,
                 float eps,
                 size_t min_size) {
  const auto n = static_cast<size_t>(points.size());

  size_t num_clusters = 0;
  labels = std::vector<int>(n, -1);  // -1 = unvisited, 0 = noise, >0 = cluster id
  for (size_t i = 0; i < n; ++i) {
    if (labels[i] != -1) {
      continue;
    }

    std::vector<size_t> neighbors;
    for (size_t j = 0; j < n; ++j) {
      if ((points[i].pos - points[j].pos).norm() <= eps) {
        neighbors.push_back(j);
      }
    }

    if (neighbors.size() < min_size) {
      labels[i] = 0;  // noise
      continue;
    }

    ++num_clusters;
    labels[i] = num_clusters;
    for (const auto q : neighbors) {
      if (labels[q] == 0) {
        labels[q] = num_clusters;
      }

      if (labels[q] != -1) {
        continue;
      }

      labels[q] = num_clusters;

      std::vector<size_t> candidates;
      for (size_t j = 0; j < n; ++j) {
        if ((points[q].pos - points[j].pos).norm() <= eps) {
          candidates.push_back(j);
        }
      }

      if (candidates.size() >= min_size) {
        neighbors.insert(neighbors.end(), candidates.begin(), candidates.end());
      }
    }
  }

  return num_clusters;
}

int getLargestCluster(const std::vector<int>& labels, size_t num_clusters) {
  size_t max_count = 0;
  int target_label = -1;
  std::vector<size_t> counts(num_clusters + 1, 0);
  for (size_t i = 0; i < labels.size(); ++i) {
    const auto label = labels[i];
    if (label <= 0) {
      continue;
    }

    const auto new_count = counts[labels[i]] + 1;
    counts[labels[i]] = new_count;
    if (new_count > max_count) {
      target_label = label;
      max_count = new_count;
    }
  }

  return target_label;
}

int getNearestCluster(const std::vector<ImagePoint>& points,
                      const std::vector<int>& labels,
                      size_t num_clusters,
                      float ratio) {
  size_t max_count = 0;
  std::vector<float> total_z(num_clusters + 1, 0.0f);
  std::vector<size_t> counts(num_clusters + 1, 0);
  for (size_t i = 0; i < labels.size(); ++i) {
    const auto label = labels[i];
    if (label <= 0) {
      continue;
    }

    const auto new_count = counts[labels[i]] + 1;
    counts[labels[i]] = new_count;
    total_z[labels[i]] += points[i].pos.z();
    if (new_count > max_count) {
      max_count = new_count;
    }
  }

  if (max_count == 0) {
    return -1;
  }

  // try to pick nearest cluster with at least threshold points
  int target_label = -1;
  float best_z = std::numeric_limits<float>::max();
  auto threshold = std::min<size_t>(static_cast<size_t>(ratio * max_count), 1u);
  if (max_count < threshold) {
    threshold = 0;  // no cluster has threshold points, fall back to picking nearest
  }

  for (size_t k = 1; k <= num_clusters; ++k) {
    if (counts[k] < threshold) {
      continue;
    }

    const auto mean_z = static_cast<float>(total_z[k] / counts[k]);
    if (mean_z < best_z) {
      best_z = mean_z;
      target_label = k;
    }
  }

  return target_label;
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

    const auto sensor = &frame->input.getSensor();
    const auto camera = dynamic_cast<const Camera*>(sensor);
    if (!camera) {
      continue;  // assumption that external object shape extraction requires images
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

    Eigen::Isometry3f input_to_cam;
    if (lidar_frame->input.points_in_world_frame) {
      input_to_cam = cam_T_world;
    } else {
      const auto world_T_lidar = lidar_frame->input.getSensorPose();
      input_to_cam = (cam_T_world * world_T_lidar.cast<float>());
    }

    const auto& vertex_map = lidar_frame->input.vertex_map;
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
  std::vector<ImagePoint> points;
  fillMaskPoints(camera, mask, depth, points);
  if (points.empty()) {
    return;
  }

  std::vector<int> labels;
  const auto num_clusters =
      runDBSCAN(points, labels, config.filter_cluster_tolerance, config.min_cluster_size);
  if (num_clusters == 0) {
    return;
  }

  int target_cluster = -1;
  if (!config.filter_prefer_near) {
    target_cluster = getLargestCluster(labels, num_clusters);
  } else {
    target_cluster = getNearestCluster(points, labels, num_clusters, config.filter_quality_ratio);
  }

  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] != target_cluster) {
      depth.at<float>(points[i].v, points[i].u) = 0.0f;
    }
  }
}

}  // namespace khronos
