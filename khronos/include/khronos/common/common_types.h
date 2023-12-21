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

#include <string>
#include <vector>

#include <glog/logging.h>
#include <hydra/common/common_types.h>
#include <hydra/openset/openset_types.h>
#include <hydra/reconstruction/voxel_types.h>
#include <hydra/utils/timing_utilities.h>
#include <opencv2/core/mat.hpp>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "khronos/common/pixel.h"

namespace khronos {
/**
 * Definition of common types that are used throughout khronos. Allthoug not all of the
 * dependencies might be needed everywhere, these core types will stay consistent.
 */

// Images.
using Image = cv::Mat;
using spark_dsg::Color;

// Time.
using hydra::fromSeconds;
using hydra::TimeStamp;
using hydra::toSeconds;

// Geometry.
using hydra::Point;
using Points = std::vector<Point>;
using Transform = Eigen::Isometry3d;

// Semantics.
using hydra::FeatureMap;
using hydra::FeatureVector;

// Index types.
using hydra::BlockIndex;
using hydra::GlobalIndex;
using hydra::VoxelIndex;
using hydra::VoxelKey;

// Index containers.
using hydra::BlockIndices;
using hydra::GlobalIndices;
using hydra::VoxelIndices;
using hydra::VoxelKeys;

// Index Sets.
using hydra::BlockIndexSet;
using hydra::GlobalIndexSet;
using hydra::VoxelIndexSet;

// Index hash maps.
using hydra::BlockIndexMap;
using hydra::GlobalIndexMap;
using hydra::VoxelIndexMap;

// Voxel-block-layer types.
using hydra::TsdfBlock;
using hydra::TsdfLayer;
using hydra::TsdfVoxel;

using hydra::SemanticBlock;
using hydra::SemanticLayer;
using hydra::SemanticVoxel;

using hydra::Mesh;
using hydra::MeshBlock;
using hydra::MeshLayer;

using hydra::TrackingBlock;
using hydra::TrackingLayer;
using hydra::TrackingVoxel;

// Scene Graph.
using spark_dsg::BoundingBox;
using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::LayerId;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphLayer;
using spark_dsg::SceneGraphNode;

// Timing.
using Timer = hydra::timing::ScopedTimer;

// Logging.
/**
 * Verbosity levels are based on frequency:
 * 1 - ONCE (e.g. on startup/shutdown)
 * 2 - INFREQUENT (e.g. on starting/stopping, high-level information)
 * 3 - EVERY FRAME
 * 4+ - DETAILS (e.g. for debugging/performance)
 */
// NOTE(lschmid): This assumes that the object using this macro has a config member
// with field verbosity to avoid duplicating this code all the time.
#define CLOG(v) LOG_IF(INFO, config.verbosity >= v)

}  // namespace khronos
