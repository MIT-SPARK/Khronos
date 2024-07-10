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

#include "khronos/active_window/graph_builder.h"

#include <config_utilities/config_utilities.h>
#include <hydra/utils/mesh_utilities.h>
#include <hydra/utils/pgmo_mesh_interface.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/compression/delta_compression.h>

#include "khronos/active_window/data/output_data.h"

namespace khronos {

using hydra::NearestNodeFinder;
using spark_dsg::NodeSymbol;

void declare_config(GraphBuilder::Config& config) {
  using namespace config;
  name("GraphBuilder");
  base<hydra::FrontendModule::Config>(config);
  field(config.verbosity, "verbosity");
}

GraphBuilder::GraphBuilder(const Config& config,
                           const hydra::SharedDsgInfo::Ptr& frontend_dsg,
                           const hydra::SharedModuleState::Ptr& state)
    : FrontendModule(config, frontend_dsg, state), config(config::checkValid(config)) {}

void GraphBuilder::initCallbacks() {
  input_callbacks_.clear();  // redefine input callback
  input_callbacks_.push_back(std::bind(&GraphBuilder::updateObjects, this, std::placeholders::_1));
  input_callbacks_.push_back(std::bind(&GraphBuilder::updateMesh, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&GraphBuilder::updateDeformationGraph, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&GraphBuilder::updatePoseGraph, this, std::placeholders::_1));
  if (hydra::GlobalInfo::instance().getConfig().enable_places) {
    input_callbacks_.push_back(std::bind(&GraphBuilder::updatePlaces, this, std::placeholders::_1));
  }
}

void GraphBuilder::updateMesh(const hydra::ReconstructionOutput& input) {
  // Archive the mesh blocks.
  Timer timer("frontend/mesh/archive", input.timestamp_ns);
  CLOG(5) << "[Graph Builder] Clearing " << input.archived_blocks.size() << " blocks from mesh";
  if (!input.archived_blocks.empty()) {
    mesh_compression_->clearArchivedBlocks(input.archived_blocks);
  }

  // Compress the new mesh.
  timer.reset("frontend/mesh/compression");
  mesh_remapping_ = std::make_shared<kimera_pgmo::HashedIndexMapping>();
  auto mesh = hydra::getActiveMesh(input.map().getMeshLayer(), input.archived_blocks);
  CLOG(5) << "[Graph Builder] Updating mesh with " << mesh->numBlocks() << " blocks";
  auto interface = hydra::PgmoMeshLayerInterface(*mesh);
  last_mesh_update_ =
      mesh_compression_->update(interface, input.timestamp_ns, mesh_remapping_.get());

  // Update the mesh in the scene graph.
  timer.reset("frontend/mesh/update");
  last_mesh_update_->updateMesh(*dsg_->graph->mesh());

  timer.reset("frontend/mesh/callbacks");
  launchCallbacks(post_mesh_callbacks_, input);
}

void GraphBuilder::updateObjects(const hydra::ReconstructionOutput& input) {
  auto khronos_data = dynamic_cast<const OutputData*>(&input);
  if (!khronos_data) {
    LOG(WARNING) << "Got input data that is not from a Khronos active window, skipping.";
    return;
  }
  std::lock_guard<std::mutex> lock(dsg_->mutex);
  Timer timer("frontend/update_objects", input.timestamp_ns);
  CLOG(5) << "[Graph Builder] Adding " << khronos_data->objects.size()
          << " objects to frontend graph.";
  // TODO(yunchang) currently objects not connected to other layers of scene graph (but is added to
  // optimization in backend)
  for (auto& object : khronos_data->objects) {
    addObjectToGraph(*object);
  }
}

void GraphBuilder::addObjectToGraph(const KhronosObjectAttributes& object) {
  NodeSymbol object_id('O', output_object_id_counter_);

  // Compute all attributes not necessarily given in the input object.
  auto attrs = std::make_unique<KhronosObjectAttributes>(object);
  attrs->color = Color::rainbowId(object.semantic_label);
  attrs->position = object.bounding_box.world_P_center.cast<double>();
  attrs->name = object_id.getLabel();
  // TODO(lschmid): Consider moving this into a proper attribute.
  attrs->details["AW_time"] = {attrs->first_observed_ns.front()};

  // Add to graph.
  dsg_->graph->emplaceNode(DsgLayers::OBJECTS, object_id, std::move(attrs));
  unarchived_objects_.insert(object_id);
  ++output_object_id_counter_;
}

void GraphBuilder::addPlaceObjectEdges(uint64_t timestamp_ns,
                                       const hydra::NodeIdSet& objects_to_add) {
  Timer timer("frontend/place_object_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  for (const auto& object_id : objects_to_add) {
    if (!dsg_->graph->hasNode(object_id)) {
      continue;
    }

    const SceneGraphNode& object_node = dsg_->graph->getNode(object_id);
    const auto parent_opt = object_node.getParent();
    if (parent_opt) {
      dsg_->graph->removeEdge(object_id, *parent_opt);
    }

    const Eigen::Vector3d object_position = dsg_->graph->getPosition(object_id);
    places_nn_finder_->find(object_position, 1, false, [&](NodeId place_id, size_t, double) {
      dsg_->graph->insertEdge(place_id, object_id);
    });
  }
}

}  // namespace khronos
