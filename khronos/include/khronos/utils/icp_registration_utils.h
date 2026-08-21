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

#include <vector>

#include <Eigen/Dense>

namespace khronos {

/**
 * @brief Utilities for ICP registration of point clouds
 */
class ICPRegistrationUtils {
 public:
  //! @brief Clone of small_gicp result structure to avoid public include
  struct Result {
    Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();
    bool converged = false;
    size_t iterations = 0;
    size_t num_inliers = 0;
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> b;
    double error = 0.0;
  };

  /**
   * @brief Perform ICP registration between two point clouds
   *
   * @param source_points Source point cloud
   * @param target_points Target point cloud
   * @param num_threads Number of threads to use for registration (default: 1)
   * @param downsampling_resolution Voxel size for downsampling (meters)
   * @param max_correspondence_distance Maximum distance for point correspondences (meters)
   * @param max_iterations Max optimizer iterations.
   * @return Registration result containing transformation and convergence info
   */
  static Result registerPointClouds(const std::vector<Eigen::Vector3f>& source_points,
                                    const std::vector<Eigen::Vector3f>& target_points,
                                    size_t num_threads = 1,
                                    float downsampling_resolution = 0.1f,
                                    float max_correspondence_distance = 0.5f,
                                    size_t max_iterations = 20);
};

}  // namespace khronos
