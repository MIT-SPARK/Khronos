#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>

#include <gtest/gtest.h>
#include <hydra/input/camera.h>
#include <hydra/input/sensor_extrinsics.h>
#include <opencv2/imgcodecs.hpp>

#include "khronos/active_window/object_extraction/object_worker_pool.h"
#include "khronos/active_window/object_extraction/track_logger.h"

namespace khronos {
namespace {

std::filesystem::path makeTempDir() {
  auto pattern = (std::filesystem::temp_directory_path() / "khronos-tracks-XXXXXX").string();
  if (!mkdtemp(pattern.data())) {
    throw std::runtime_error("could not create temporary directory");
  }
  return pattern;
}

std::shared_ptr<hydra::Camera> makeCamera() {
  hydra::Camera::Config config;
  config.min_range = 0.01;
  config.max_range = 100.0;
  config.fx = 10.0f;
  config.fy = 10.0f;
  config.cx = 2.0f;
  config.cy = 1.5f;
  config.width = 4;
  config.height = 3;
  config.extrinsics = hydra::IdentitySensorExtrinsics::Config{};
  return std::make_shared<hydra::Camera>(config, "test_camera");
}

// Frame with one semantic cluster (id 1) covering the given pixels.
FrameData::Ptr makeFrame(const std::shared_ptr<hydra::Camera>& camera,
                         TimeStamp stamp,
                         const Pixels& pixels,
                         const std::optional<Eigen::Isometry3d>& pose = std::nullopt) {
  hydra::InputData input(camera);
  input.timestamp_ns = stamp;
  input.world_T_body = Eigen::Isometry3d::Identity();
  input.world_T_body.translation() << 1.0, 2.0, 3.0;
  if (pose) {
    input.world_T_body = *pose;
  }
  // RGB: pure red
  input.color_image = cv::Mat(3, 4, CV_8UC3, cv::Scalar(255, 0, 0));
  input.depth_image = cv::Mat(3, 4, CV_32FC1, cv::Scalar(1.5f));
  input.depth_image.at<float>(0, 0) = std::numeric_limits<float>::quiet_NaN();
  auto frame = std::make_shared<FrameData>(input);
  MeasurementCluster cluster;
  cluster.id = 1;
  cluster.pixels = pixels;
  frame->semantic_clusters.push_back(cluster);
  return frame;
}

std::string readFile(const std::filesystem::path& path) {
  std::ifstream fin(path);
  std::stringstream ss;
  ss << fin.rdbuf();
  return ss.str();
}

struct FailingExtractor : ObjectExtractor {
  KhronosObjectAttributes::Ptr extractObject(const Track&, const FrameDataBuffer&) override {
    return nullptr;
  }
};

}  // namespace

TEST(TrackLogger, EncodeMaskRle) {
  cv::Mat mask = cv::Mat::zeros(2, 3, CV_8UC1);
  mask.at<uint8_t>(0, 1) = 255;
  mask.at<uint8_t>(0, 2) = 255;
  mask.at<uint8_t>(1, 2) = 255;
  EXPECT_EQ(TrackLogger::encodeMaskRle(mask), "[1,2,2,1]");
  mask.at<uint8_t>(0, 0) = 255;
  EXPECT_EQ(TrackLogger::encodeMaskRle(mask), "[0,3,2,1]");
  EXPECT_EQ(TrackLogger::encodeMaskRle(cv::Mat::zeros(2, 2, CV_8UC1)), "[4]");
}

TEST(TrackLogger, DisabledWithoutPath) {
  TrackLogger logger(TrackLogger::Config{});
  EXPECT_FALSE(logger.enabled());
  logger.log(Track{}, FrameDataBuffer(FrameDataBuffer::Config{}), true);
  logger.flush();
}

TEST(TrackLogger, WritesTrackAndKeptImages) {
  const auto root = makeTempDir();
  const auto camera = makeCamera();

  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  buffer.storeData(makeFrame(camera, 10, {{1, 1}}));                  // 1 pixel
  buffer.storeData(makeFrame(camera, 20, {{1, 0}, {2, 0}, {2, 1}}));  // 3 pixels -> kept
  buffer.storeData(makeFrame(camera, 30, {{0, 2}}));                  // no observation

  Track track;
  track.id = 12;
  track.absorbed_track_ids = {9, 7};
  track.observations.emplace_back(10, 1, -1, "test_camera");
  track.observations.emplace_back(20, 1, -1, "test_camera");
  track.observations.emplace_back(25, 1, -1, "test_camera");   // frame not in buffer
  track.observations.emplace_back(30, -1, -1, "test_camera");  // no semantic cluster
  track.semantics = SemanticClusterInfo(3);

  TrackLogger::Config config;
  config.log_path = root / "tracks";
  config.save_depth = true;
  {
    TrackLogger logger(config);
    ASSERT_TRUE(logger.enabled());
    logger.log(track, buffer, true);
    logger.flush();
  }

  const auto track_dir = root / "tracks" / "12";
  const auto json = readFile(track_dir / "track.json");
  EXPECT_NE(json.find("{\"version\": 1, \"robot_id\": 0, \"track_id\": 12, "
                      "\"absorbed_track_ids\": [7, 9], \"extracted\": true, "
                      "\"semantic_label\": 3, \"observations\": ["),
            std::string::npos)
      << json;
  // non-kept observation: metadata only plus RLE mask
  EXPECT_NE(json.find("{\"k\": 0, \"timestamp_ns\": 10, \"world_T_body\": [1, 0, 0, 1, 0, 1, 0, "
                      "2, 0, 0, 1, 3, 0, 0, 0, 1], \"sensor\": {\"name\": \"test_camera\", "
                      "\"width\": 4, \"height\": 3, \"fx\": 10, \"fy\": 10, \"cx\": 2, \"cy\": "
                      "1.5, \"body_T_sensor\": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, "
                      "1]}, \"bbox_2d\": [1, 1, 1, 1], \"mask_rle\": \"[5,1,6]\", \"score\": 1, "
                      "\"kept\": false, \"color\": null, \"depth\": null, \"mask\": null}"),
            std::string::npos)
      << json;
  EXPECT_NE(json.find("{\"k\": 1, \"timestamp_ns\": 20"), std::string::npos) << json;
  EXPECT_NE(json.find("\"bbox_2d\": [1, 0, 2, 1], \"mask_rle\": null, \"score\": 3, \"kept\": "
                      "true, \"color\": \"obs_1_color.jpg\", \"depth\": \"obs_1_depth.png\", "
                      "\"mask\": \"obs_1_mask.png\"}"),
            std::string::npos)
      << json;
  EXPECT_EQ(json.find("\"k\": 2"), std::string::npos) << json;

  EXPECT_FALSE(std::filesystem::exists(track_dir / "obs_0_color.jpg"));
  const auto color = cv::imread((track_dir / "obs_1_color.jpg").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(color.size(), cv::Size(4, 3));
  const auto pixel = color.at<cv::Vec3b>(1, 1);  // BGR on disk: red
  EXPECT_GT(pixel[2], 200);
  EXPECT_LT(pixel[0], 50);

  const auto mask = cv::imread((track_dir / "obs_1_mask.png").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(mask.type(), CV_8UC1);
  ASSERT_EQ(mask.size(), cv::Size(4, 3));
  EXPECT_EQ(mask.at<uint8_t>(0, 1), 255);
  EXPECT_EQ(mask.at<uint8_t>(1, 2), 255);
  EXPECT_EQ(mask.at<uint8_t>(1, 1), 0);

  const auto depth = cv::imread((track_dir / "obs_1_depth.png").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(depth.type(), CV_16UC1);
  EXPECT_EQ(depth.at<uint16_t>(1, 1), 1500);
  EXPECT_EQ(depth.at<uint16_t>(0, 0), 0);

  for (const auto& entry : std::filesystem::directory_iterator(track_dir)) {
    EXPECT_EQ(entry.path().string().find(".tmp"), std::string::npos) << entry.path();
  }

  std::filesystem::remove_all(root);
}

TEST(TrackLogger, LogsFailedExtraction) {
  const auto root = makeTempDir();
  TrackLogger::Config config;
  config.log_path = root;
  auto logger = std::make_shared<TrackLogger>(config);

  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  buffer.storeData(makeFrame(makeCamera(), 10, {{1, 1}}));
  Track track;
  track.id = 4;
  track.observations.emplace_back(10, 1, -1, "test_camera");

  {
    ObjectWorkerPool pool(ObjectWorkerPool::Config{}, std::make_unique<FailingExtractor>(), logger);
    pool.submit(0, track, buffer);
    pool.join();
  }
  {  // without an extractor tracks are still logged (as not extracted)
    ObjectWorkerPool pool(ObjectWorkerPool::Config{}, nullptr, logger);
    track.id = 5;
    pool.submit(0, track, buffer);
  }
  logger->flush();

  for (const auto id : {"4", "5"}) {
    const auto json = readFile(root / id / "track.json");
    EXPECT_NE(json.find("\"extracted\": false"), std::string::npos) << json;
    EXPECT_NE(json.find("\"semantic_label\": null"), std::string::npos) << json;
    // dropped tracks are metadata only: no kept observation and no image files
    EXPECT_EQ(json.find("\"kept\": true"), std::string::npos) << json;
    EXPECT_NE(json.find("\"kept\": false"), std::string::npos) << json;
    for (const auto& entry : std::filesystem::directory_iterator(root / id)) {
      EXPECT_EQ(entry.path().filename(), "track.json") << entry.path();
    }
  }

  std::filesystem::remove_all(root);
}

TEST(TrackLogger, KeepsImagesOfDroppedTracksWhenConfigured) {
  const auto root = makeTempDir();
  TrackLogger::Config config;
  config.log_path = root;
  config.keep_images_of_dropped_tracks = true;
  auto logger = std::make_shared<TrackLogger>(config);

  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  buffer.storeData(makeFrame(makeCamera(), 10, {{1, 1}}));
  Track track;
  track.id = 6;
  track.observations.emplace_back(10, 1, -1, "test_camera");
  {
    ObjectWorkerPool pool(ObjectWorkerPool::Config{}, std::make_unique<FailingExtractor>(), logger);
    pool.submit(0, track, buffer);
    pool.join();
  }
  logger->flush();

  const auto json = readFile(root / "6" / "track.json");
  EXPECT_NE(json.find("\"extracted\": false"), std::string::npos) << json;
  EXPECT_NE(json.find("\"kept\": true"), std::string::npos) << json;
  std::filesystem::remove_all(root);
}

TEST(TrackLogger, NonFiniteValuesAreNull) {
  const auto root = makeTempDir();
  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 3.0;
  buffer.storeData(makeFrame(makeCamera(), 10, {{1, 1}}, pose));
  Track track;
  track.id = 8;
  track.observations.emplace_back(10, 1, -1, "test_camera");

  TrackLogger::Config config;
  config.log_path = root;
  {
    TrackLogger logger(config);
    logger.log(track, buffer, true);
    logger.flush();
  }

  const auto json = readFile(root / "8" / "track.json");
  EXPECT_NE(json.find("\"world_T_body\": [1, 0, 0, null, 0, 1, 0, null, 0, 0, 1, 3, 0, 0, 0, 1]"),
            std::string::npos)
      << json;
  EXPECT_EQ(json.find("nan"), std::string::npos) << json;
  EXPECT_EQ(json.find("inf"), std::string::npos) << json;
  std::filesystem::remove_all(root);
}

TEST(TrackLogger, DropsTracksWhenQueueIsFull) {
  const auto root = makeTempDir();
  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  buffer.storeData(makeFrame(makeCamera(), 10, {{1, 1}}));

  TrackLogger::Config config;
  config.log_path = root;
  config.max_queue_size = 2;
  {
    TrackLogger logger(config);
    logger.pause(true);  // hold the writer so the queue fills up
    for (int id = 1; id <= 4; ++id) {
      Track track;
      track.id = id;
      track.observations.emplace_back(10, 1, -1, "test_camera");
      logger.log(track, buffer, true);
    }

    EXPECT_EQ(logger.numDropped(), 2u);
    logger.pause(false);
    logger.flush();
  }

  // the first max_queue_size tracks were written, the rest dropped
  EXPECT_TRUE(std::filesystem::exists(root / "1" / "track.json"));
  EXPECT_TRUE(std::filesystem::exists(root / "2" / "track.json"));
  EXPECT_FALSE(std::filesystem::exists(root / "3"));
  EXPECT_FALSE(std::filesystem::exists(root / "4"));
  std::filesystem::remove_all(root);
}

}  // namespace khronos
