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

#include <opencv2/core/mat.hpp>

namespace khronos {

/**
 * @brief Pixel coordinates in an image.
 */
struct Pixel {
  int u;
  int v;
  Pixel() = default;
  Pixel(int _u, int _v) : u(_u), v(_v) {}

  // Operators.
  Pixel operator+(const Pixel& other) const { return Pixel(u + other.u, v + other.v); }
  Pixel operator-(const Pixel& other) const { return Pixel(u - other.u, v - other.v); }
  bool operator<(const Pixel& other) const { return u < other.u || (u == other.u && v < other.v); }

  // Utilities.
  bool isInImage(const cv::Mat& image) const {
    return u >= 0 && u < image.cols && v >= 0 && v < image.rows;
  }

  std::vector<Pixel> neighbors4() const { return {{u + 1, v}, {u, v + 1}, {u - 1, v}, {u, v - 1}}; }

  std::vector<Pixel> neighbors8() const {
    return {{u + 1, v},
            {u, v + 1},
            {u - 1, v},
            {u, v - 1},
            {u + 1, v + 1},
            {u + 1, v - 1},
            {u - 1, v + 1},
            {u - 1, v - 1}};
  }
};
using Pixels = std::vector<Pixel>;

}  // namespace khronos
