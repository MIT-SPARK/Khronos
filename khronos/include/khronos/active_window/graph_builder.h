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

#pragma once

#include <memory>
#include <unordered_set>
#include <utility>

#include <hydra/common/global_info.h>
#include <hydra/frontend/frontend_module.h>

#include "khronos/common/common_types.h"

namespace khronos {

class GraphBuilder : public hydra::FrontendModule {
 public:
  using Ptr = std::shared_ptr<GraphBuilder>;
  using ConstPtr = std::shared_ptr<const GraphBuilder>;

  struct Config : public hydra::FrontendModule::Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;
  } const config;

  GraphBuilder(const Config& config,
               const hydra::SharedDsgInfo::Ptr& frontend_dsg,
               const hydra::SharedModuleState::Ptr& state);
  virtual ~GraphBuilder() = default;

  // Set/get the input queue.
  FrontendInputQueue::Ptr getQueue() const { return queue_; }
  void setQueue(FrontendInputQueue::Ptr queue) { queue_ = std::move(queue); }

  // Override from hydra::FrontendModule.
  void initCallbacks() override;

 protected:
  // Khronos specific callbacks.
  void updateMesh(const hydra::ReconstructionOutput& input);

  void updateObjects(const hydra::ReconstructionOutput& input);

  // Helper methods.
  void addPlaceObjectEdges(uint64_t timestamp_ns, const hydra::NodeIdSet& objects_to_add);

  void addObjectToGraph(const KhronosObjectAttributes& object);

 private:
  // Enumerate object ids when adding to the graph.
  size_t output_object_id_counter_ = 0;

  std::unordered_set<NodeId> unarchived_objects_;
};

void declare_config(GraphBuilder::Config& config);

}  // namespace khronos
