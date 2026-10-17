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

#include <atomic>
#include <memory>

#include <config_utilities/config_utilities.h>

namespace khronos {

/**
 * @brief Run control of the active window change detector: whether detection is enabled and
 * whether its history should be reset. Thread safe; the detector polls it at the start of each
 * frame. This base class has no external interface, subclasses (e.g. ROS services) drive it.
 */
class ChangeDetectionController {
 public:
  using Ptr = std::unique_ptr<ChangeDetectionController>;

  struct Config {};

  explicit ChangeDetectionController(const Config& config);
  virtual ~ChangeDetectionController() = default;

  //! Enable or disable detection. Returns false if it already was in that state.
  bool setEnabled(bool enabled);
  bool isEnabled() const { return enabled_; }

  //! Request a history reset, applied by the detector at its next frame.
  void requestReset() { reset_requested_ = true; }
  //! Returns whether a reset was requested and clears the request.
  bool consumeReset() { return reset_requested_.exchange(false); }

 protected:
  //! Called after every setEnabled, e.g. to publish the state.
  virtual void onStateChanged(bool /* enabled */) {}

 private:
  std::atomic<bool> enabled_{true};
  std::atomic<bool> reset_requested_{false};
};

void declare_config(ChangeDetectionController::Config& config);

}  // namespace khronos
