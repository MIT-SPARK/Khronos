#pragma once
#include <set>

#include <hydra/input/camera.h>
#include <ianvs/node_handle.h>
#include <khronos/active_window/object_extraction/object_extractor.h>
#include <khronos_msgs/srv/extract_implicit_object.hpp>
#include <spark_dsg/node_attributes.h>

namespace khronos {

using spark_dsg::KhronosObjectAttributes;

class ExternalObjectExtractor : public ObjectExtractor {
 public:
  using InferenceSrv = khronos_msgs::srv::ExtractImplicitObject;
  using Frames = std::vector<const FrameData*>;

  struct Config {
    //! Minimum track confidence to run external shape extractor
    float min_object_allocation_confidence = 0.5f;
    //! Minimum points required for a cluster
    size_t min_cluster_size = 10;
    //! Set of labels to ignore
    std::set<int32_t> excluded_labels;
    //! Names of sensors to drop observations from
    std::set<std::string> excluded_sensors;
    //! LiDAR depth projection settings
    enum class DepthMode : int {
      LIDAR_ONLY,
      REPLACE_CAMERA,
      CAMERA_ONLY
    } depth_mode = DepthMode::CAMERA_ONLY;
    //! Toggles between filling extraction mask from observation or instance image
    bool mask_from_cluster = true;
    //! Enable post-projection depth filtering
    bool filter_projected_depth = false;
    //! DBSCAN eps: max inter-point distance (m)
    float filter_cluster_tolerance = 0.3f;
    //! Keep the nearest cluster if true (otherwise furthest)
    bool filter_prefer_near = true;
    //! Min fraction of largest cluster size to qualify as a candidate
    float filter_quality_ratio = 0.3f;
  } const config;

  explicit ExternalObjectExtractor(Config& config);
  virtual ~ExternalObjectExtractor() = default;

  virtual KhronosObjectAttributes::Ptr extractObject(const Track& track,
                                                     const FrameDataBuffer& buffer);

 protected:
  struct InstanceResult {
    double volume = 0;
    int cluster_id = -1;
    const FrameData* frame = nullptr;
    const MeasurementCluster* cluster = nullptr;
  };
  InstanceResult getBestCluster(const Track& track,
                                const FrameDataBuffer& buffer,
                                Frames& lidar_frames) const;

  cv::Mat createLidarOnlyDepthImage(const FrameData& cam_frame,
                                    const Frames& lidar_frames,
                                    const hydra::Camera& camera) const;

  cv::Mat replaceLidarDepthImage(const FrameData& cam_frame,
                                 const Frames& lidar_frames,
                                 const hydra::Camera& camera) const;

  cv::Mat projectLidarPoints(const FrameData& cam_frame,
                             const Frames& lidar_frames,
                             const hydra::Camera& camera,
                             cv::Mat base_depth) const;

  void filterDepthByCluster(cv::Mat& depth, const cv::Mat& mask, const hydra::Camera& camera) const;

 private:
  ianvs::NodeHandle nh_;
  rclcpp::Client<InferenceSrv>::SharedPtr client_;
};

}  // namespace khronos
