#include "khronos_ros/active_window/external_object_extractor.h"

#include <memory>

#include <Eigen/Dense>
#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <cv_bridge/cv_bridge.hpp>
#include <hydra/input/camera.h>
#include <khronos/active_window/data/measurement_clusters.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hydra_multi_system {

using namespace std::chrono_literals;
using khronos::FrameData;
using khronos::FrameDataBuffer;
using khronos::MeasurementCluster;
using khronos::Track;

namespace {

static const auto registration =
    config::RegistrationWithConfig<khronos::ObjectExtractor,
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

}  // namespace

void declare_config(ExternalObjectExtractor::Config& config) {
  using namespace config;
  name("ExternalObjectExtractor::Config");
  field(config.min_object_allocation_confidence, "min_object_allocation_confidence");
  field(config.excluded_labels, "excluded_labels");
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

  int best_cluster_id = -1;
  double best_volume = 0.0;
  FrameData::ConstPtr best_frame = nullptr;
  for (const auto& obs : track.observations) {
    const auto frame = buffer.getData(obs.stamp);
    if (!frame) {
      continue;
    }

    const auto cluster = findClusterForId(*frame, obs.semantic_cluster_id);
    if (!cluster) {
      continue;
    }

    const auto curr_volume = cluster->bounding_box.volume();
    if (curr_volume > best_volume) {
      best_frame = frame;
      best_volume = curr_volume;
      best_cluster_id = obs.semantic_cluster_id;
    }
  }

  if (!best_frame) {
    LOG(WARNING) << "Invalid track " << track.id << ": no observation in buffer!";
    return nullptr;
  }

  const auto base_ptr = &(best_frame->input.getSensor());
  const auto cam_ptr = dynamic_cast<const hydra::Camera*>(base_ptr);
  if (!cam_ptr) {
    LOG(ERROR) << "Camera sensor required!";
    return nullptr;
  }

  const auto best_cluster = findClusterForId(*best_frame, best_cluster_id);
  if (best_cluster->pixels.size() < 5) {
    LOG(ERROR) << "Too few pixels in best cluster: " << best_cluster->pixels.size() << " < 5";
    return nullptr;
  }

  if (!best_cluster->semantics) {
    LOG(ERROR) << "Semantics required for mesh extraction!";
    return nullptr;
  }

  if (config.excluded_labels.count(best_cluster->semantics->category_id)) {
    VLOG(1) << "Skipping excluded label for track " << track.id;
    return nullptr;
  }

  bool first = true;
  Eigen::Vector2i min;
  Eigen::Vector2i max;
  for (const auto& pixel : best_cluster->pixels) {
    const Eigen::Vector2i p_curr(pixel.u, pixel.v);
    if (first) {
      min = p_curr;
      max = p_curr;
      first = false;
    } else {
      min = min.cwiseMin(p_curr);
      max = max.cwiseMax(p_curr);
    }
  }

  // const Eigen::Vector2i dims = max - min + Eigen::Vector2i::Ones();
  min = Eigen::Vector2i::Zero();
  const Eigen::Vector2i dims(best_frame->input.color_image.cols,
                             best_frame->input.color_image.rows);
  const cv::Rect roi(min.x(), min.y(), dims.x(), dims.y());
  VLOG(1) << "Got ROI: min=[" << min.x() << ", " << min.y() << "], max=[" << dims.x() << ", "
          << dims.y() << "]";

  cv::Mat mask = cv::Mat::zeros(dims.y(), dims.x(), CV_8UC1);
  for (const auto& pixel : best_cluster->pixels) {
    const int local_v = pixel.v - min.y();
    const int local_u = pixel.u - min.x();
    CHECK_LT(local_v, mask.rows);
    CHECK_LT(local_u, mask.cols);
    CHECK_GE(local_v, 0);
    CHECK_GE(local_u, 0);
    mask.at<uint8_t>(local_v, local_u) = 255;
  }

  auto req = std::make_shared<InferenceSrv::Request>();
  // fillMessage(req->rgb, best_frame->input.color_image, "rgb8", roi);
  // fillMessage(req->depth, best_frame->input.depth_image, "32FC1", roi);
  fillMessage(req->rgb, best_frame->input.color_image, "rgb8");
  fillMessage(req->depth, best_frame->input.depth_image, "32FC1");
  fillMessage(req->mask, mask, "8UC1");

  // We send a cropped image, but we want the intrinsics to be the same as
  // the full image. The crop is just a translation in the image plane,
  // so we subtract out the translation (e.g., if the crop starts at the center pixel,
  // there should be no translation component in the camera matrix)
  const auto& cam_config = cam_ptr->getConfig();
  req->intrinsics.k[0] = cam_config.fx;
  req->intrinsics.k[4] = cam_config.fy;
  req->intrinsics.k[2] = cam_config.cx - min.x();
  req->intrinsics.k[5] = cam_config.cy - min.y();
  req->volume = best_volume;
  req->semantic_id = best_cluster->semantics->category_id;

  const auto rep = ianvs::call_service(*client_, req);
  if (!rep) {
    LOG(ERROR) << "CRISP service call failed!";
    return nullptr;
  }

  // NOTE(hlim): Without this line,
  // if an invalid crisp value is returned, the mesh visualization no longer works.
  if (!rep->valid) {
    return nullptr;
  }

  Eigen::Isometry3d cam_T_nocs;
  tf2::convert(rep->pose, cam_T_nocs);
  Eigen::Quaterniond cam_q_nocs(cam_T_nocs.rotation());
  const Eigen::Isometry3d world_T_cam = best_frame->input.getSensorPose();
  const Eigen::Isometry3d world_T_nocs = world_T_cam * cam_T_nocs;

  auto attrs = std::make_unique<KhronosObjectAttributes>();

  attrs->registered = true;
  attrs->position = world_T_nocs.translation();
  attrs->world_R_object = world_T_nocs.rotation();
  attrs->last_update_time_ns = best_frame->input.timestamp_ns;
  attrs->is_active = false;
  attrs->metadata.set({{"scale", rep->scale}});
  attrs->bounding_box = spark_dsg::BoundingBox(Eigen::Vector3f::Constant(rep->scale),
                                               attrs->position.cast<float>().eval(),
                                               attrs->world_R_object.cast<float>());
  attrs->semantic_label = best_cluster->semantics->category_id;
  attrs->semantic_feature =
      Eigen::Map<const Eigen::VectorXf>(rep->shape_code.data(), rep->shape_code.size());

  VLOG(5) << "Got response features of " << rep->shape_code.size() << " elements";
  return attrs;
}

}  // namespace hydra_multi_system
