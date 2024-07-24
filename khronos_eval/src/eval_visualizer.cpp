#include "khronos_eval/eval_visualizer.h"

#include <filesystem>

#include <hydra/utils/csv_reader.h>
#include <khronos_ros/visualization/visualization_utils.h>

namespace khronos {

EvalVisualizer::EvalVisualizer(const ros::NodeHandle& nh)
    : config(nh, nh.resolveName("khronos/eval_vis")), nh_(nh) {
  // ROS.
  centroid_gt_pub_ = nh_.advertise<visualization_msgs::Marker>("centroids_gt", 1);
  centroid_dsg_pub_ = nh_.advertise<visualization_msgs::Marker>("centroids_dsg", 1);
  bbox_gt_pub_ = nh_.advertise<visualization_msgs::Marker>("boundingboxes_gt", 1);
  bbox_dsg_pub_ = nh_.advertise<visualization_msgs::Marker>("boundingboxes_dsg", 1);
  association_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("associations", 1);
  object_id_gt_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_ids_gt", 1);
  object_id_dsg_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_ids_dsg", 1);

  // Use the same cached header throughout.
  header_.frame_id = "world";

  // Load the data.
  loadData();

  // Register dynamic reconfigure.
  config.setUpdateCallback([this]() { reset(); });
}

void EvalVisualizer::spin() {
  // Continuously publish in case things change.
  while (ros::ok()) {
    draw();
    ros::spinOnce();
  }
}

void EvalVisualizer::reset() {
  // Clear all markers to force re-creation of markers.
  std::lock_guard<std::mutex> lock(mutex_);
  centroid_gt_marker_.reset();
  centroid_dsg_marker_.reset();
  bbox_gt_marker_.reset();
  bbox_dsg_marker_.reset();
  association_marker_.reset();
  object_id_gt_marker_.reset();
  object_id_dsg_marker_.reset();
}

void EvalVisualizer::draw() {
  std::lock_guard<std::mutex> lock(mutex_);
  header_.stamp = ros::Time::now();

  // Get the right data.
  size_t robot_time_idx = config.get().robot_time;
  size_t query_time_idx = config.get().query_time;
  if (config.get().robot_time != previous_robot_time_idx_ ||
      config.get().query_time != previous_query_time_idx_) {
    previous_robot_time_idx_ = config.get().robot_time;
    previous_query_time_idx_ = config.get().query_time;
    if (robot_time_idx >= data_.size()) {
      LOG(WARNING) << "Robot time index " << robot_time_idx
                   << " is out of bounds (max: " << data_.size() - 1 << ")";
      robot_time_idx = data_.size() - 1;
    }
    const size_t num_query_times = data_[robot_time_idx].size();
    if (query_time_idx >= num_query_times) {
      LOG(WARNING) << "Query time index " << query_time_idx
                   << " is out of bounds (max: " << num_query_times - 1 << ") for robot time "
                   << robot_time_idx << ".";
      query_time_idx = num_query_times - 1;
    }
    LOG(INFO) << "Displaying data for robot time " << robot_time_idx << " ("
              << times_seconds_[robot_time_idx] << "s) and query time " << query_time_idx << " ("
              << times_seconds_[query_time_idx] << "s).";
  }

  const auto& data = data_[robot_time_idx][query_time_idx];

  // Draw things.
  drawCentroids(data);
  drawBboxes(data);
  drawAssociations(data);
  drawObjectIds(data);
}

void EvalVisualizer::drawCentroids(const VisData& data) {
  if (centroid_gt_pub_.getNumSubscribers() == 0 && centroid_dsg_pub_.getNumSubscribers() == 0) {
    return;
  }

  if (!centroid_gt_marker_) {
    // Create the markers.
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale = setScale(config.get().centroid_scale);
    marker.pose.orientation.w = 1.0;
    marker.header = header_;
    marker.action = visualization_msgs::Marker::ADD;
    auto coloring_fn = getObjectColoringFunction();

    // Add DSG.
    for (const auto& [_, object] : data.dsg_objects) {
      marker.points.push_back(setPoint(object.centroid));
      marker.colors.push_back(setColor(coloring_fn(object)));
    }
    if (marker.points.empty()) {
      marker.points.emplace_back();
    }
    centroid_dsg_marker_ = std::make_unique<visualization_msgs::Marker>(marker);

    // Add GT.
    marker.points.clear();
    marker.colors.clear();
    const Point offset = Point(0, 0, config.get().z_offset);
    for (const auto& [_, object] : data.gt_objects) {
      marker.points.push_back(setPoint(object.centroid + offset));
      marker.colors.push_back(setColor(coloring_fn(object)));
    }
    if (marker.points.empty()) {
      marker.points.emplace_back();
    }
    centroid_gt_marker_ = std::make_unique<visualization_msgs::Marker>(marker);
  }

  // Publish the markers.
  if (centroid_gt_pub_.getNumSubscribers() > 0) {
    centroid_gt_pub_.publish(*centroid_gt_marker_);
  }
  if (centroid_dsg_pub_.getNumSubscribers() > 0) {
    centroid_dsg_pub_.publish(*centroid_dsg_marker_);
  }
}

void EvalVisualizer::drawBboxes(const VisData& data) {
  if (bbox_gt_pub_.getNumSubscribers() == 0 && bbox_dsg_pub_.getNumSubscribers() == 0) {
    return;
  }

  if (!bbox_gt_marker_) {
    // Create the markers.
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale = setScale(config.get().bbox_scale);
    marker.pose.orientation.w = 1.0;
    marker.header = header_;
    marker.action = visualization_msgs::Marker::ADD;
    auto coloring_fn = getObjectColoringFunction();

    // Add the DSG.
    for (const auto& [_, object] : data.dsg_objects) {
      const Color color = coloring_fn(object);
      const auto bbox_marker = setBoundingBox(object.bbox, color, header_);
      marker.points.insert(
          marker.points.end(), bbox_marker.points.begin(), bbox_marker.points.end());
      marker.colors.insert(marker.colors.end(), bbox_marker.points.size(), setColor(color));
    }
    if (marker.points.empty()) {
      marker.points.emplace_back();
      marker.points.emplace_back();
    }
    bbox_dsg_marker_ = std::make_unique<visualization_msgs::Marker>(marker);

    // Add GT.
    marker.points.clear();
    marker.colors.clear();
    const Point offset = Point(0, 0, config.get().z_offset);
    for (const auto& [_, object] : data.gt_objects) {
      const Color color = coloring_fn(object);
      BoundingBox bbox = object.bbox;
      bbox.world_P_center += offset;
      const auto bbox_marker = setBoundingBox(bbox, color, header_);
      marker.points.insert(
          marker.points.end(), bbox_marker.points.begin(), bbox_marker.points.end());
      marker.colors.insert(marker.colors.end(), bbox_marker.points.size(), setColor(color));
    }
    if (marker.points.empty()) {
      marker.points.emplace_back();
      marker.points.emplace_back();
    }
    bbox_gt_marker_ = std::make_unique<visualization_msgs::Marker>(marker);
  }

  // Publish the markers.
  if (bbox_gt_pub_.getNumSubscribers() > 0) {
    bbox_gt_pub_.publish(*bbox_gt_marker_);
  }
  if (bbox_dsg_pub_.getNumSubscribers() > 0) {
    bbox_dsg_pub_.publish(*bbox_dsg_marker_);
  }
}

void EvalVisualizer::drawAssociations(const VisData& data) {
  if (!association_pub_.getNumSubscribers()) {
    return;
  }

  if (!association_marker_) {
    // Create the markers.
    association_marker_ = std::make_unique<visualization_msgs::MarkerArray>();
    visualization_msgs::Marker msg;
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.scale.x = config.get().association_scale;
    msg.pose.orientation.w = 1.f;
    msg.header = header_;
    const Point offset = Point(0, 0, config.get().z_offset);

    // Get all associations setup.
    std::unordered_map<std::string, int> num_association_in_dsg, num_association_in_gt;
    for (const auto& [from, to] : data.dsg_to_gt) {
      num_association_in_gt[to]++;
    }
    for (const auto& [from, to] : data.gt_to_dsg) {
      num_association_in_dsg[to]++;
    }

    // Visualize correct associations.
    msg.color = setColor(Color(0, 200, 0));
    msg.ns = "correct";
    for (const auto& [from, to] : data.dsg_to_gt) {
      if (num_association_in_dsg[from] != 1 || num_association_in_gt[to] != 1) {
        continue;
      }
      msg.points.push_back(setPoint(data.dsg_objects.at(from).centroid));
      msg.points.push_back(setPoint(data.gt_objects.at(to).centroid + offset));
    }
    if (msg.points.empty()) {
      msg.points.emplace_back();
      msg.points.emplace_back();
    }
    association_marker_->markers.push_back(msg);

    // Visualize under segmentation.
    msg.color = setColor(Color(50, 50, 255));
    msg.points.clear();
    msg.ns = "undersegmented";
    for (const auto& [from, to] : data.gt_to_dsg) {
      if (num_association_in_dsg[to] <= 1) {
        continue;
      }
      msg.points.push_back(setPoint(data.gt_objects.at(from).centroid + offset));
      msg.points.push_back(setPoint(data.dsg_objects.at(to).centroid));
    }
    if (msg.points.empty()) {
      msg.points.emplace_back();
      msg.points.emplace_back();
    }
    association_marker_->markers.push_back(msg);

    // Visualize over segmentation.
    msg.color = setColor(Color(255, 50, 50));
    msg.points.clear();
    msg.ns = "oversegmented";
    for (const auto& [from, to] : data.dsg_to_gt) {
      if (num_association_in_gt[to] <= 1) {
        continue;
      }
      msg.points.push_back(setPoint(data.dsg_objects.at(from).centroid));
      msg.points.push_back(setPoint(data.gt_objects.at(to).centroid + offset));
    }
    if (msg.points.empty()) {
      msg.points.emplace_back();
      msg.points.emplace_back();
    }
    association_marker_->markers.push_back(msg);
  }

  // Publish the markers.
  association_pub_.publish(*association_marker_);
}

void EvalVisualizer::drawObjectIds(const VisData& data) {
  if (object_id_gt_pub_.getNumSubscribers() == 0) {
    return;
  }
  if (!object_id_gt_marker_) {
    // Create the markers.
    object_id_gt_marker_ = std::make_unique<visualization_msgs::MarkerArray>();
    object_id_dsg_marker_ = std::make_unique<visualization_msgs::MarkerArray>();

    visualization_msgs::Marker msg;
    msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg.scale.z = config.get().id_scale;
    msg.pose.orientation.w = 1.f;
    msg.header = header_;
    msg.color.a = 1.f;
    msg.action = visualization_msgs::Marker::ADD;
    int id = 0;
    const float z_offset = (config.get().centroid_scale + config.get().id_scale) / 2.f + 0.05f;

    // DSG.
    for (const auto& [_, object] : data.dsg_objects) {
      msg.id = id++;
      msg.text = object.id;
      msg.pose.position = setPoint(object.centroid);
      msg.pose.position.z += z_offset;
      object_id_dsg_marker_->markers.push_back(msg);
    }
    size_t num_id_markers = object_id_dsg_marker_->markers.size();
    for (size_t i = num_id_markers; i < num_previous_id_dsg_markers_; ++i) {
      msg.id = id++;
      msg.action = visualization_msgs::Marker::DELETE;
      object_id_dsg_marker_->markers.push_back(msg);
    }
    num_previous_id_dsg_markers_ = num_id_markers;

    // GT.
    for (const auto& [_, object] : data.gt_objects) {
      msg.id = id++;
      msg.text = object.id;
      msg.pose.position = setPoint(object.centroid);
      msg.pose.position.z += z_offset + config.get().z_offset;
      object_id_gt_marker_->markers.push_back(msg);
    }
    num_id_markers = object_id_gt_marker_->markers.size();
    for (size_t i = num_id_markers; i < num_previous_id_gt_markers_; ++i) {
      msg.id = id++;
      msg.action = visualization_msgs::Marker::DELETE;
      object_id_gt_marker_->markers.push_back(msg);
    }
    num_previous_id_gt_markers_ = num_id_markers;
  }

  // Publish the markers.
  if (object_id_gt_pub_.getNumSubscribers() > 0) {
    object_id_gt_pub_.publish(*object_id_gt_marker_);
  }
  if (object_id_dsg_pub_.getNumSubscribers() > 0) {
    object_id_dsg_pub_.publish(*object_id_dsg_marker_);
  }
}

std::function<Color(const VisObject&)> EvalVisualizer::getObjectColoringFunction() const {
  if (config.get().object_color == 1) {
    // Presence.
    return [](const VisObject& object) {
      if (object.is_present) {
        return Color(0, 0, 255);
      }
      return Color(100, 100, 100);
    };
  } else if (config.get().object_color == 2) {
    // Persistence.
    return [](const VisObject& object) {
      if (object.is_present) {
        if (object.has_appeared) {
          return Color(0, 225, 0);  // Appeared.
        } else {
          return Color(0, 0, 255);  // Persistent.
        }
      } else {
        if (object.has_disappeared) {
          return Color(255, 0, 0);  // Disappeared.
        } else {
          return Color(100, 100, 100);  // Absent.
        }
      }
    };
  } else if (config.get().object_color != 0) {
    LOG(WARNING) << "Unknown object coloring function " << config.get().object_color
                 << ". Using default.";
  }

  // Default: Label
  return [this](const VisObject& object) { return Color::rainbowId(object.label); };
}

void EvalVisualizer::loadData() {
  // Get the file names and check if they exist.
  std::string visualization_directory;
  nh_.getParam("visualization_directory", visualization_directory);
  if (visualization_directory.empty()) {
    LOG(FATAL) << "Param 'visualization_directory' not set.";
  }
  const std::string objects_file = visualization_directory + "/objects.csv";
  if (!std::filesystem::exists(objects_file)) {
    LOG(FATAL) << "File '" << objects_file << "' does not exist.";
  }
  const std::string associations_file = visualization_directory + "/associations.csv";
  if (!std::filesystem::exists(associations_file)) {
    LOG(FATAL) << "File '" << associations_file << "' does not exist.";
  }
  LOG(INFO) << "Loading Visualization data from '" << visualization_directory << "'...";

  // Load the data.
  hydra::CsvReader objects_reader(objects_file);
  if (!objects_reader) {
    LOG(FATAL) << "Could not read '" << objects_file << "'.";
  }
  hydra::CsvReader associations_reader(associations_file);
  if (!associations_reader) {
    LOG(FATAL) << "Could not read '" << associations_file << "'.";
  }

  // Parse objects.
  std::unordered_map<std::string, size_t> map_to_idx;
  std::unordered_map<uint64_t, size_t> time_to_idx;
  std::unordered_set<int> labels;
  for (const auto& row : objects_reader.getRows()) {
    // Allocate the map stamp if needed.
    const std::string& map_name = row.getEntry("MapName");
    auto it = map_to_idx.find(map_name);
    if (it == map_to_idx.end()) {
      it = map_to_idx.insert(std::pair(map_name, data_.size())).first;
      data_.emplace_back();
    }
    auto& data = data_[it->second];

    // Allocate the time stamp if needed.
    const uint64_t timestamp = std::stoull(row.getEntry("QueryTime"));
    auto it2 = time_to_idx.find(timestamp);
    if (it2 == time_to_idx.end()) {
      it2 = time_to_idx.insert(std::pair(timestamp, times_.size())).first;
      times_.emplace_back(timestamp);
    }

    // NOTE(lschmid): This assumes that the time stamps are sorted, which they are when created
    // by the object evaluator.
    const size_t time_idx = it2->second;
    if (data.size() <= time_idx) {
      data.resize(time_idx + 1);
    }
    auto& vis_data = data[time_idx];

    // Populate the data.
    VisObject object;
    object.id = row.getEntry("ID");
    object.label = std::stoi(row.getEntry("Label"));
    labels.insert(object.label);
    object.centroid.x() = std::stof(row.getEntry("CentroidX"));
    object.centroid.y() = std::stof(row.getEntry("CentroidY"));
    object.centroid.z() = std::stof(row.getEntry("CentroidZ"));
    object.bbox.world_P_center.x() = std::stof(row.getEntry("BBPosX"));
    object.bbox.world_P_center.y() = std::stof(row.getEntry("BBPosY"));
    object.bbox.world_P_center.z() = std::stof(row.getEntry("BBPosZ"));
    object.bbox.dimensions.x() = std::stof(row.getEntry("BBDimX"));
    object.bbox.dimensions.y() = std::stof(row.getEntry("BBDimY"));
    object.bbox.dimensions.z() = std::stof(row.getEntry("BBDimZ"));
    object.is_present = std::stoi(row.getEntry("Present"));
    object.has_appeared = std::stoi(row.getEntry("HasAppeared"));
    object.has_disappeared = std::stoi(row.getEntry("HasDisappeared"));
    const bool is_gt = std::stoi(row.getEntry("IsGT"));

    if (is_gt) {
      vis_data.gt_objects.emplace(object.id, std::move(object));
    } else {
      vis_data.dsg_objects.emplace(object.id, std::move(object));
    }
  }

  // Parse associations.
  for (const auto& row : associations_reader.getRows()) {
    // Allocate the map stamp if needed.
    const std::string& map_name = row.getEntry("MapName");
    auto& data = data_[map_to_idx.at(map_name)];
    // NOTE: This assumes that the time stamps are sorted, which they are when created by the
    // object evaluator.
    const uint64_t timestamp = std::stoull(row.getEntry("QueryTime"));
    auto& vis_data = data[time_to_idx.at(timestamp)];

    // Populate the data.
    const std::string& to_id = row.getEntry("ToID");
    if (to_id == "Failed") {
      continue;
    }
    const std::string& from_id = row.getEntry("FromID");
    const bool from_gt = std::stoi(row.getEntry("FromGT"));

    if (from_gt) {
      vis_data.gt_to_dsg.emplace(from_id, to_id);
    } else {
      vis_data.dsg_to_gt.emplace(from_id, to_id);
    }
  }

  // Compute timestamps seconds.
  times_seconds_.reserve(times_.size());
  for (const size_t stamp : times_) {
    times_seconds_.emplace_back(static_cast<float>(stamp - times_[0]) / 1e9f);
  }
  LOG(INFO) << "Loaded " << data_.size() << " maps with " << times_.size() << " time stamps.";
}

}  // namespace khronos
