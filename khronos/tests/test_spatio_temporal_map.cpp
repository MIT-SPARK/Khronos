/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <hydra/utils/temp_path.h>
#include <spark_dsg/node_attributes.h>

#include "khronos/spatio_temporal_map/spatio_temporal_map.h"

namespace khronos {
namespace {

DynamicSceneGraph::Ptr makeGraph() {
  return DynamicSceneGraph::fromNames({{DsgLayers::AGENTS, 7}, {DsgLayers::OBJECTS, 7}});
}

}  // namespace

TEST(SpatioTemporalMap, SavesAndLoadsMeshlessSnapshotAtTimeZero) {
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-map-", true);
  for (const auto incrementally : {true, false}) {
    SpatioTemporalMap::Config config;
    config.finalize_incrementally = incrementally;
    SpatioTemporalMap map(config);
    map.update(makeGraph(), 0);
    map.finalize();
    const auto path = (temp.path / "meshless.4dmap").string();
    ASSERT_TRUE(map.save(path));
    const auto loaded = SpatioTemporalMap::load(path);
    ASSERT_TRUE(loaded);
    ASSERT_EQ(loaded->numTimeSteps(), 1u);
    const auto graph = loaded->getDsgPtr(0);
    ASSERT_TRUE(graph);
    EXPECT_FALSE(graph->hasMesh());
    EXPECT_EQ(graph->numNodes(), 0u);
  }
}

TEST(SpatioTemporalMap, RevealsAndTrimsRobotPartitionsWithoutTreatingObjectsAsAgents) {
  auto graph = makeGraph();
  graph->setMesh(std::make_shared<spark_dsg::Mesh>());
  for (const auto robot : {'c', 'f'}) {
    for (size_t index = 0; index < 3; ++index) {
      const spark_dsg::NodeSymbol id(robot, index);
      const auto stamp = std::chrono::nanoseconds(10 * (index + 1));
      auto attrs = std::make_unique<spark_dsg::AgentNodeAttributes>(
          stamp, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), id);
      ASSERT_TRUE(graph->emplaceNode(7, id, std::move(attrs), robot));
    }
  }

  const spark_dsg::NodeSymbol object('O', 0);
  auto attrs = std::make_unique<KhronosObjectAttributes>();
  attrs->first_observed_ns = {5};
  ASSERT_TRUE(graph->emplaceNode(7, object, std::move(attrs)));
  SpatioTemporalMap map(SpatioTemporalMap::Config{});
  map.update(graph, 5);
  map.update(graph->clone(), 40);

  for (const TimeStamp stamp : {10, 30, 20, 5, 30}) {
    const auto current = map.getDsgPtr(stamp);
    ASSERT_TRUE(current);
    EXPECT_TRUE(current->hasNode(object));
    for (const auto robot : {'c', 'f'}) {
      for (size_t index = 0; index < 3; ++index) {
        const spark_dsg::NodeSymbol id(robot, index);
        EXPECT_EQ(current->hasNode(id), 10 * (index + 1) <= stamp);
      }
    }
  }
}

}  // namespace khronos
