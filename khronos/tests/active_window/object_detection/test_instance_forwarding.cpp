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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/common/global_info.h>
#include <khronos/active_window/object_detection/instance_forwarding.h>

namespace khronos {
namespace {

struct ConfigGuard {
  ConfigGuard(bool init = true) {
    if (init) {
      hydra::PipelineConfig config;
      hydra::GlobalInfo::init(config);
    }
  }

  ~ConfigGuard() { hydra::GlobalInfo::instance().reset(); }
};

Eigen::VectorXf getOneHot(size_t index, size_t dims = 10) {
  Eigen::VectorXf vec = Eigen::VectorXf::Zero(dims);
  vec(index) = 1.0f;
  return vec;
}

}  // namespace

void PrintTo(const SemanticClusterInfo& info, std::ostream* os) {
  const Eigen::IOFormat fmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
  *os << "(id=" << info.category_id << ", vec" << info.feature.format(fmt) << ")";
}

TEST(InstanceForwarding, CategoryFilterCorrect) {
  ConfigGuard guard;

  CategoryFilter::Config config;
  config.invalid = {1, 2, 3};
  CategoryFilter filter(config);

  {  // no labels means everything fails
    hydra::InputData input{nullptr};
    const FrameData data{input};
    EXPECT_FALSE(filter.valid(data, 1, {{1, 2}}));
  }

  {  // no pixels means cluster fails
    InputData input{nullptr};
    input.label_image = cv::Mat(3, 2, CV_32SC1);
    input.label_image.at<InputData::LabelType>(2, 1) = 2;

    const FrameData data{input};
    EXPECT_FALSE(filter.valid(data, 1, {}));
  }

  {  // invalid label means instance fails
    InputData input{nullptr};
    input.label_image = cv::Mat(3, 2, CV_32SC1);
    input.label_image.at<InputData::LabelType>(2, 1) = 2;

    const FrameData data{input};
    EXPECT_FALSE(filter.valid(data, 1, {{1, 2}}));
  }

  {  // valid label means instance passes
    InputData input{nullptr};
    input.label_image = cv::Mat(3, 2, CV_32SC1);
    input.label_image.at<InputData::LabelType>(2, 1) = 4;

    const FrameData data{input};
    EXPECT_TRUE(filter.valid(data, 1, {{1, 2}}));
  }
}

TEST(InstanceForwarding, OpenVocabFilterCorrect) {
  ConfigGuard guard;

  hydra::EmbeddingGroup group;
  group.embeddings = {getOneHot(0), getOneHot(1), getOneHot(2)};
  OpenVocabBackgroundFilter::Config config;
  config.background = group;
  OpenVocabBackgroundFilter filter(config);

  {  // no feature means everything fails
    hydra::InputData input{nullptr};
    input.label_features = {{1, getOneHot(1)}};
    const FrameData data{input};
    EXPECT_FALSE(filter.valid(data, 2, {{1, 2}}));
  }

  {  // feature match means ID fails
    hydra::InputData input{nullptr};
    input.label_features = {{1, getOneHot(1)}};
    const FrameData data{input};
    EXPECT_FALSE(filter.valid(data, 1, {{1, 2}}));
  }

  {  // feature mismatch means ID passes
    hydra::InputData input{nullptr};
    input.label_features = {{1, getOneHot(4)}};
    const FrameData data{input};
    EXPECT_TRUE(filter.valid(data, 1, {{1, 2}}));
  }
}

TEST(InstanceForwarding, EmptyInputCorrect) {
  InstanceForwarding::Config config;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  {  // no input means no output
    hydra::InputData input{nullptr};
    FrameData data{input};
    detector.processInput(map, data);
    EXPECT_TRUE(data.object_image.empty());
    EXPECT_TRUE(data.semantic_clusters.empty());
  }

  {  // no instances means no output
    hydra::InputData input{nullptr};
    input.instance_image = cv::Mat::zeros(6, 4, CV_16S);
    input.range_image = cv::Mat(6, 4, CV_32FC1, 1.0f);
    FrameData data{input};
    data.object_image = cv::Mat::zeros(6, 4, CV_32S);

    detector.processInput(map, data);
    EXPECT_TRUE(data.semantic_clusters.empty());
  }
}

TEST(InstanceForwarding, ZeroIndexedInstance) {
  InstanceForwarding::Config config;
  config.zero_is_unlabeled = false;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  hydra::InputData input{nullptr};
  input.instance_image = cv::Mat::zeros(3, 2, CV_16S);
  input.range_image = cv::Mat(3, 2, CV_32FC1, 1.0f);
  FrameData data{input};
  data.object_image = cv::Mat::zeros(3, 2, CV_32S);

  detector.processInput(map, data);
  ASSERT_EQ(data.semantic_clusters.size(), 1u);
  const auto& cluster = data.semantic_clusters[0];
  Pixels expected{{0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}};
  EXPECT_EQ(cluster.pixels, expected);
  EXPECT_EQ(cluster.semantics, std::nullopt);
}

TEST(InstanceForwarding, OutOfRange) {
  InstanceForwarding::Config config;
  config.zero_is_unlabeled = false;
  config.min_range = 1.5f;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  hydra::InputData input{nullptr};
  input.instance_image = cv::Mat::zeros(3, 2, CV_16S);
  input.range_image = cv::Mat(3, 2, CV_32FC1, 1.0f);
  FrameData data{input};
  data.object_image = cv::Mat::zeros(3, 2, CV_32S);

  detector.processInput(map, data);
  EXPECT_EQ(data.semantic_clusters.size(), 0u);
}

TEST(InstanceForwarding, PartialRange) {
  InstanceForwarding::Config config;
  config.zero_is_unlabeled = false;
  config.min_range = 0.5f;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  hydra::InputData input{nullptr};
  input.instance_image = cv::Mat::zeros(3, 2, CV_16S);
  input.range_image = cv::Mat(3, 2, CV_32FC1, 1.0f);
  input.range_image.at<InputData::RangeType>(1, 1) = 0.1f;
  input.range_image.at<InputData::RangeType>(2, 0) = 0.1f;

  FrameData data{input};
  data.object_image = cv::Mat::zeros(3, 2, CV_32S);

  detector.processInput(map, data);
  ASSERT_EQ(data.semantic_clusters.size(), 1u);
  const auto& cluster = data.semantic_clusters[0];
  Pixels expected{{0, 0}, {0, 1}, {1, 0}, {1, 2}};
  EXPECT_EQ(cluster.pixels, expected);
  EXPECT_EQ(cluster.semantics, std::nullopt);
}

TEST(InstanceForwarding, ClosedSetLabels) {
  InstanceForwarding::Config config;
  config.zero_is_unlabeled = false;
  config.min_range = 0.5f;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  hydra::InputData input{nullptr};
  input.instance_image = cv::Mat::zeros(3, 2, CV_16S);
  input.label_image = cv::Mat::zeros(3, 2, CV_32S);
  input.label_image = 2;
  input.range_image = cv::Mat(3, 2, CV_32FC1, 1.0f);
  input.range_image.at<InputData::RangeType>(1, 1) = 0.1f;
  input.range_image.at<InputData::RangeType>(2, 0) = 0.1f;

  FrameData data{input};
  data.object_image = cv::Mat::zeros(3, 2, CV_32S);

  detector.processInput(map, data);
  ASSERT_EQ(data.semantic_clusters.size(), 1u);
  const auto& cluster = data.semantic_clusters[0];
  Pixels expected{{0, 0}, {0, 1}, {1, 0}, {1, 2}};
  EXPECT_EQ(cluster.pixels, expected);
  const SemanticClusterInfo expected_semantics{2};
  EXPECT_EQ(cluster.semantics, expected_semantics);
}

TEST(InstanceForwarding, OpenSetFeatures) {
  InstanceForwarding::Config config;
  config.zero_is_unlabeled = false;
  config.min_range = 0.5f;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  hydra::InputData input{nullptr};
  input.instance_image = cv::Mat::zeros(3, 2, CV_16S);
  input.instance_image.at<InputData::InstanceType>(0, 0) = 1;
  input.label_features = {{1, getOneHot(2)}};
  input.range_image = cv::Mat(3, 2, CV_32FC1, 1.0f);
  input.range_image.at<InputData::RangeType>(1, 1) = 0.1f;
  input.range_image.at<InputData::RangeType>(2, 0) = 0.1f;

  FrameData data{input};
  data.object_image = cv::Mat::zeros(3, 2, CV_32S);

  detector.processInput(map, data);
  ASSERT_EQ(data.semantic_clusters.size(), 2u);

  {  // first instance has no feature
    const auto& cluster = data.semantic_clusters[0];
    Pixels expected{{0, 1}, {1, 0}, {1, 2}};
    EXPECT_EQ(cluster.pixels, expected);
    EXPECT_EQ(cluster.semantics, std::nullopt);
  }

  {
    const auto& cluster = data.semantic_clusters[1];
    Pixels expected{{0, 0}};
    EXPECT_EQ(cluster.pixels, expected);
    const SemanticClusterInfo expected_semantics{getOneHot(2)};
    EXPECT_EQ(cluster.semantics, expected_semantics);
  }
}

TEST(InstanceForwarding, FilteringCorrect) {
  ConfigGuard guard;

  InstanceForwarding::Config config;
  config.zero_is_unlabeled = false;
  config.min_range = 0.5f;
  config.min_object_volume = 0.1;
  config.min_cluster_size = 2;

  CategoryFilter::Config filter_config;
  filter_config.invalid = {1};
  config.instance_filter = filter_config;
  InstanceForwarding detector(config);

  hydra::VolumetricMap map(hydra::VolumetricMap::Config{});

  // four instances: first fails pixels, second volume, third label filter
  hydra::InputData input{nullptr};
  input.instance_image = cv::Mat::zeros(4, 2, CV_16S);
  input.instance_image.at<InputData::InstanceType>(1, 0) = 1;
  input.instance_image.at<InputData::InstanceType>(1, 1) = 1;
  input.instance_image.at<InputData::InstanceType>(2, 0) = 2;
  input.instance_image.at<InputData::InstanceType>(2, 1) = 2;
  input.instance_image.at<InputData::InstanceType>(3, 0) = 3;
  input.instance_image.at<InputData::InstanceType>(3, 1) = 3;

  input.label_image = cv::Mat::zeros(4, 2, CV_32S);
  input.label_image.at<InputData::LabelType>(2, 0) = 1;
  input.label_image.at<InputData::LabelType>(2, 1) = 1;
  input.label_image.at<InputData::LabelType>(3, 0) = 2;
  input.label_image.at<InputData::LabelType>(3, 1) = 2;

  input.range_image = cv::Mat(4, 2, CV_32FC1, 1.0f);
  input.range_image.at<InputData::RangeType>(0, 1) = 0.1f;

  input.vertex_map = cv::Mat(4, 2, CV_32FC3, cv::Scalar(0.0f, 0.0f, 0.0f));
  input.vertex_map.at<cv::Vec3f>(2, 1) = cv::Vec3f(1.0f, 1.0f, 1.0f);
  input.vertex_map.at<cv::Vec3f>(3, 1) = cv::Vec3f(1.0f, 1.0f, 1.0f);

  FrameData data{input};
  data.object_image = cv::Mat::zeros(4, 2, CV_32S);
  detector.processInput(map, data);

  ASSERT_EQ(data.semantic_clusters.size(), 1u);
  const auto& cluster = data.semantic_clusters[0];
  Pixels expected{{0, 3}, {1, 3}};
  EXPECT_EQ(cluster.pixels, expected);
  const SemanticClusterInfo expected_semantics{2};
  EXPECT_EQ(cluster.semantics, expected_semantics);
}

}  // namespace khronos
