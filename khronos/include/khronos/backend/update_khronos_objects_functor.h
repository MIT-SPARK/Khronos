#pragma once
#include <config_utilities/factory.h>
#include <hydra/backend/association_strategies.h>
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/update_functions.h>
#include <hydra/utils/active_window_tracker.h>

#include "khronos/common/common_types.h"
namespace khronos {
using hydra::MergeList;
using hydra::SharedDsgInfo;
using hydra::UpdateInfo;
using spark_dsg::LayerView;
struct UpdateKhronosObjectsFunctor : public hydra::UpdateFunctor {
  struct Config {
    //! Number of control points for deforming the places (if not in optimization)
    size_t num_control_points = 4;
    //! Timestamp tolerance when deforming the places (if not in optimization)
    double control_point_tolerance_s = 10.0;
    //! Require merges to have same semantic label
    bool merge_require_same_label = true;
    //! Require merges to not be co-visible
    bool merge_require_no_co_visibility = false;
    //! Min IOU to be considered a merge
    double merge_min_iou = 0.5;
    //! Association strategy for finding matches to active nodes
    hydra::MergeProposer::Config merge_proposer = {
        config::VirtualConfig<hydra::AssociationStrategy>{
            hydra::association::SemanticNearestNode::Config{}}};
  } const config;

  explicit UpdateKhronosObjectsFunctor(const Config& config);
  Hooks hooks() const override;
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;
  void interpFromValues(const LayerView& view,
                        SharedDsgInfo& dsg,
                        const UpdateInfo::ConstPtr& info) const;

  MergeList findMerges(const DynamicSceneGraph& graph, const UpdateInfo::ConstPtr& info) const;

  const hydra::MergeProposer merge_proposer;

  mutable hydra::ActiveWindowTracker active_tracker;
  mutable std::unordered_map<NodeId, Eigen::Vector3d> cached_pos_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdateKhronosObjectsFunctor, Config>(
          "UpdateKhronosObjectsFunctor");
};

void declare_config(UpdateKhronosObjectsFunctor::Config& config);

}  // namespace khronos
