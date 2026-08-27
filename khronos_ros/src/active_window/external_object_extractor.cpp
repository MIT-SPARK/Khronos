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
#include <khronos/utils/geometry_utils.h>
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

void fillMaskFromCluster(const MeasurementCluster& cluster, cv::Mat& mask) {
  for (const auto& [u, v] : cluster.pixels) {
    mask.at<uint8_t>(v, u) = 255;
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

void fillMaskPoints(const hydra::Camera& camera,
                    const cv::Mat& mask,
                    const cv::Mat& depth,
                    Points& points,
                    Pixels& pixels) {
  for (int r = 0; r < depth.rows; ++r) {
    for (int c = 0; c < depth.cols; ++c) {
      if (mask.at<uint8_t>(r, c) == 0) {
        continue;
      }

      const auto d = depth.at<float>(r, c);
      if (!std::isfinite(d) || d <= 0.f) {
        continue;
      }

      points.push_back(camera.unprojectPixel(c, r, d));
      pixels.emplace_back(c, r);
    }
  }
}

// This file's own cluster-selection policy: among the DBSCAN, pick the one closest to the camera
// by mean depth. Only clusters holding at least `ratio` of the largest cluster's point count are
// eligible; if none qualify, every cluster is considered. Returns utils::kDbscanNoise if there are
// no clusters.
int getNearestCluster(const Points& points, const std::vector<int>& labels, float ratio) {
  const auto sizes = utils::dbscanClusterSizes(labels);
  if (sizes.empty()) {
    return utils::kDbscanNoise;
  }

  std::vector<float> total_z(sizes.size(), 0.0f);
  size_t max_count = 0;
  for (size_t i = 0; i < labels.size(); ++i) {
    const auto label = labels[i];
    if (label == utils::kDbscanNoise) {
      continue;
    }
    total_z[label] += points[i].z();
    max_count = std::max(max_count, sizes[label]);
  }

  auto threshold = std::max<size_t>(static_cast<size_t>(ratio * max_count), 1u);
  if (max_count < threshold) {
    threshold = 0;  // no cluster meets the ratio; fall back to considering all of them
  }

  int target_label = utils::kDbscanNoise;
  float best_z = std::numeric_limits<float>::max();
  for (size_t k = 0; k < sizes.size(); ++k) {
    if (sizes[k] < threshold) {
      continue;
    }

    const auto mean_z = static_cast<float>(total_z[k] / sizes[k]);
    if (mean_z < best_z) {
      best_z = mean_z;
      target_label = static_cast<int>(k);
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
  field(config.excluded_sensors, "excluded_sensors");
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

  std::vector<const FrameData*> lidar_frames;
  const auto best = getBestCluster(track, buffer, lidar_frames);
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

auto ExternalObjectExtractor::getBestCluster(const Track& track,
                                             const FrameDataBuffer& buffer,
                                             Frames& lidar_frames) const -> InstanceResult {
  InstanceResult best;
  for (const auto& obs : track.observations) {
    const auto frame = buffer.getData(obs.stamp, obs.sensor);
    if (!frame) {
      continue;
    }

    const auto cluster = findClusterForId(*frame, obs.semantic_cluster_id);
    if (!cluster) {
      continue;
    }

    const auto sensor = &frame->input.getSensor();
    if (config.excluded_sensors.count(sensor->name)) {
      continue;
    }

    const auto camera = dynamic_cast<const Camera*>(sensor);
    if (!camera) {
      if (config.depth_mode != Config::DepthMode::CAMERA_ONLY) {
        lidar_frames.push_back(frame.get());
      }

      continue;  // assumption that external object shape extraction requires images
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
  Points points;
  Pixels pixels;
  fillMaskPoints(camera, mask, depth, points, pixels);
  if (points.empty()) {
    return;
  }

  const auto labels = utils::dbscan(
      points, config.filter_cluster_tolerance, static_cast<int>(config.min_cluster_size));

  const int target_cluster = config.filter_prefer_near
                                 ? getNearestCluster(points, labels, config.filter_quality_ratio)
                                 : utils::largestDbscanClusterLabel(labels);
  if (target_cluster == utils::kDbscanNoise) {
    return;
  }

  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] != target_cluster) {
      depth.at<float>(pixels[i].v, pixels[i].u) = 0.0f;
    }
  }
}

}  // namespace khronos
