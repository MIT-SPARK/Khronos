#include "khronos/backend/update_khronos_objects_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/backend/backend_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/mesh_utilities.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>
namespace khronos {

using spark_dsg::KhronosObjectAttributes;

struct AttributeMap {
  std::vector<KhronosObjectAttributes*> attributes;

  void push_back(KhronosObjectAttributes* attrs) { attributes.push_back(attrs); }

  void sort() {
    std::sort(attributes.begin(), attributes.end(), [](const auto& lhs, const auto& rhs) {
      return lhs->first_observed_ns.front() < rhs->first_observed_ns.front();
    });
  }
};

size_t pgmoNumVertices(const AttributeMap& map) { return map.attributes.size(); }

kimera_pgmo::traits::Pos pgmoGetVertex(const AttributeMap& map,
                                       size_t i,
                                       kimera_pgmo::traits::VertexTraits* traits) {
  const auto& attrs = map.attributes[i];
  if (traits) {
    traits->stamp = attrs->first_observed_ns.front();
  }

  return attrs->position.cast<float>();
}

uint64_t pgmoGetVertexStamp(const AttributeMap& map, size_t i) {
  return map.attributes[i]->first_observed_ns.front();
}

void pgmoSetVertex(AttributeMap& map,
                   size_t i,
                   const kimera_pgmo::traits::Pos& pos,
                   const kimera_pgmo::traits::VertexTraits&) {
  map.attributes[i]->position = pos.cast<double>();
  map.attributes[i]->bounding_box.world_P_center = pos.cast<float>();
}

void declare_config(UpdateKhronosObjectsFunctor::Config& config) {
  using namespace config;
  name("UpdateKhronosObjectsFunctor::Config");
  field(config.num_control_points, "num_control_points");
  field(config.control_point_tolerance_s, "control_point_tolerance_s", "s");
  field(config.merge_proposer, "merge_proposer");
  field(config.merge_require_same_label, "merge_require_same_label");
  field(config.merge_require_no_co_visibility, "merge_require_no_co_visibility");
  field(config.merge_min_iou, "merge_min_iou");
}

UpdateKhronosObjectsFunctor::UpdateKhronosObjectsFunctor(const Config& config)
    : config(config::checkValid(config)), merge_proposer(config.merge_proposer) {}

hydra::UpdateFunctor::Hooks UpdateKhronosObjectsFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.find_merges = [this](const auto& graph, const auto& info) {
    return findMerges(graph, info);
  };

  return my_hooks;
}

void UpdateKhronosObjectsFunctor::interpFromValues(const LayerView& view,
                                                   SharedDsgInfo& dsg,
                                                   const UpdateInfo::ConstPtr& info) const {
  if (!info->deformation_graph) {
    return;
  }

  const auto this_robot = hydra::GlobalInfo::instance().getRobotPrefix().id;

  std::map<char, AttributeMap> nodes;
  for (const auto& node : view) {
    size_t robot_id = this_robot;
    if (info->node_to_robot_id) {
      auto iter = info->node_to_robot_id->find(node.id);
      if (iter == info->node_to_robot_id->end()) {
        LOG(WARNING) << "Node " << NodeSymbol(node.id) << " does not belong to robot";
      } else {
        robot_id = iter->second;
      }
    }

    const auto prefix = kimera_pgmo::GetVertexPrefix(robot_id);
    auto robot_attrs = nodes.find(prefix);
    if (robot_attrs == nodes.end()) {
      robot_attrs = nodes.emplace(prefix, AttributeMap{}).first;
    }

    auto& attrs = node.attributes<KhronosObjectAttributes>();
    auto cache_iter = cached_pos_.find(node.id);
    if (cache_iter == cached_pos_.end()) {
      cache_iter = cached_pos_.emplace(node.id, attrs.position).first;
    } else if (attrs.is_active) {
      // update cache if node is still active
      cache_iter->second = attrs.position;
    }

    // set the position of the node to the original position before deformation
    attrs.position = cache_iter->second;
    robot_attrs->second.push_back(&attrs);
  }

  auto& dgraph = *info->deformation_graph;
  for (auto& [prefix, attributes] : nodes) {
    if (!dgraph.hasVertexKey(prefix)) {
      continue;
    }

    const auto& control_points = dgraph.getInitialPositionsVertices(prefix);
    if (control_points.size() < config.num_control_points) {
      continue;
    }

    attributes.sort();  // make sure attributes are sorted by timestamp
    std::vector<std::set<size_t>> vertex_graph_map_deformed;
    kimera_pgmo::deformation::deformPoints(attributes,
                                           vertex_graph_map_deformed,
                                           attributes,
                                           prefix,
                                           control_points,
                                           dgraph.getVertexStamps(prefix),
                                           *dgraph.getValues(),
                                           config.num_control_points,
                                           config.control_point_tolerance_s,
                                           nullptr);
  }

  for (const auto& node : view) {
    dsg.graph->setNodeAttributes(node.id, node.attributes().clone());
  }
}

void UpdateKhronosObjectsFunctor::call(const DynamicSceneGraph& unmerged,
                                       SharedDsgInfo& dsg,
                                       const UpdateInfo::ConstPtr& info) const {
  Timer spin_timer("backend/update_khronos_objects", info->timestamp_ns);
  if (!unmerged.hasLayer(DsgLayers::OBJECTS)) {
    VLOG(5) << "Skipping khronos object update due to missing layer";
    return;
  }

  // we want to use the unmerged graph for most things
  const auto& objects = unmerged.getLayer(DsgLayers::OBJECTS);
  // we want to iterate over the unmerged graph
  const auto new_loopclosure = info->loop_closure_detected;
  active_tracker.clear();  // reset from previous pass
  LayerView view = new_loopclosure ? LayerView(objects) : active_tracker.view(objects);

  // interpolate to update
  interpFromValues(view, dsg, info);
}

MergeList UpdateKhronosObjectsFunctor::findMerges(const DynamicSceneGraph& graph,
                                                  const UpdateInfo::ConstPtr& info) const {
  if (!graph.hasLayer(DsgLayers::OBJECTS)) {
    return {};
  }

  const auto new_lcd = info->loop_closure_detected;
  const auto& objects = graph.getLayer(DsgLayers::OBJECTS);
  // freeze layer view to avoid messing with tracker
  LayerView view = new_lcd ? LayerView(objects) : active_tracker.view(objects, true);

  MergeList proposals;
  merge_proposer.findMerges(
      objects,
      view,
      [this](const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
        const auto lhs_attrs = lhs.tryAttributes<spark_dsg::KhronosObjectAttributes>();
        const auto rhs_attrs = rhs.tryAttributes<spark_dsg::KhronosObjectAttributes>();

        if (!lhs_attrs || !rhs_attrs) {
          return false;
        }

        if (config.merge_require_same_label) {
          if (lhs_attrs->semantic_label != rhs_attrs->semantic_label) {
            return false;
          }
        }

        if (config.merge_require_no_co_visibility) {
          if (lhs_attrs->first_observed_ns.front() < rhs_attrs->first_observed_ns.front() &&
              lhs_attrs->last_observed_ns.front() > rhs_attrs->first_observed_ns.front()) {
            return false;
          }

          if (lhs_attrs->last_observed_ns.front() > rhs_attrs->first_observed_ns.front() &&
              lhs_attrs->first_observed_ns.front() < rhs_attrs->last_observed_ns.front()) {
            return false;
          }
        }

        if (!lhs_attrs->bounding_box.intersects(rhs_attrs->bounding_box)) {
          return false;
        }

        return config.merge_min_iou == 0 ||
               lhs_attrs->bounding_box.computeIoU(rhs_attrs->bounding_box) >= config.merge_min_iou;
      },
      proposals);
  return proposals;
}

}  // namespace khronos
