#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <map>
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

// All files below a directory, by relative path
std::map<std::string, std::string> readTree(const std::filesystem::path& root) {
  std::map<std::string, std::string> result;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      std::ifstream fin(entry.path(), std::ios::binary);
      std::stringstream ss;
      ss << fin.rdbuf();
      result[std::filesystem::relative(entry.path(), root).string()] = ss.str();
    }
  }
  return result;
}

// Feeds frames through a buffer of the given size the way the active window does (snapshot every
// dropped frame for the live tracks), then logs the tracks as extracted.
std::map<std::string, std::string> logWithBuffer(TrackLogger::Config config,
                                                 size_t max_buffer_size,
                                                 const std::vector<FrameData::Ptr>& frames,
                                                 const Tracks& tracks,
                                                 size_t* num_evicted = nullptr) {
  const auto root = makeTempDir();
  config.log_path = root;
  FrameDataBuffer::Config buffer_config;
  buffer_config.max_buffer_size = max_buffer_size;
  FrameDataBuffer buffer(buffer_config);
  {
    TrackLogger logger(config);
    for (const auto& frame : frames) {
      const auto evicted = buffer.storeData(frame);
      if (evicted) {
        logger.snapshot(*evicted, tracks);
      }
    }

    if (num_evicted) {
      *num_evicted = logger.numEvicted();
    }

    for (const auto& track : tracks) {
      logger.log(track, buffer, true, logger.takeEvicted(track));
    }
    logger.flush();
    EXPECT_EQ(logger.numEvicted(), 0u);
    EXPECT_EQ(logger.evictedBytes(), 0u);
  }

  auto result = readTree(root);
  std::filesystem::remove_all(root);
  return result;
}

Pixels makePixels(size_t num) {
  Pixels pixels;
  for (size_t i = 0; i < num; ++i) {
    pixels.emplace_back(static_cast<int>(i % 4), static_cast<int>(i / 4));
  }
  return pixels;
}

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

TEST(TrackLogger, EvictedFrameKeepsImageAndObservation) {
  const auto camera = makeCamera();
  // the best observation (3 pixels) is the oldest frame, which the buffer drops
  const std::vector<FrameData::Ptr> frames{makeFrame(camera, 10, makePixels(3)),
                                           makeFrame(camera, 20, makePixels(1)),
                                           makeFrame(camera, 30, makePixels(2))};
  Track track;
  track.id = 3;
  for (const TimeStamp stamp : {10, 20, 30}) {
    track.observations.emplace_back(stamp, 1, -1, "test_camera");
  }

  TrackLogger::Config config;
  config.save_depth = true;
  size_t num_evicted = 0;
  const auto evicted = logWithBuffer(config, 2, frames, {track}, &num_evicted);
  EXPECT_EQ(num_evicted, 1u);
  ASSERT_TRUE(evicted.count("3/track.json"));
  const auto& json = evicted.at("3/track.json");
  EXPECT_NE(json.find("{\"k\": 0, \"timestamp_ns\": 10, "), std::string::npos) << json;
  EXPECT_NE(json.find("\"bbox_2d\": [0, 0, 2, 0], \"mask_rle\": null, \"score\": 3, \"kept\": "
                      "true, \"color\": \"obs_0_color.jpg\", \"depth\": \"obs_0_depth.png\", "
                      "\"mask\": \"obs_0_mask.png\"}"),
            std::string::npos)
      << json;
  EXPECT_NE(json.find("{\"k\": 2, \"timestamp_ns\": 30"), std::string::npos) << json;
  for (const auto file : {"3/obs_0_color.jpg", "3/obs_0_depth.png", "3/obs_0_mask.png"}) {
    EXPECT_TRUE(evicted.count(file)) << file;
  }

  // same files, byte for byte, as with a buffer that never drops a frame
  EXPECT_EQ(evicted, logWithBuffer(config, 100, frames, {track}));
}

TEST(TrackLogger, EvictedRankingEqualsUnboundedBuffer) {
  const auto camera = makeCamera();
  // scores (pixel counts) with ties, best ones spread over old and recent frames
  const std::vector<size_t> sizes{2, 7, 1, 7, 3, 12, 5, 7, 2, 9, 12, 4};
  std::vector<FrameData::Ptr> frames;
  Track first, second;
  first.id = 1;
  second.id = 2;
  for (size_t i = 0; i < sizes.size(); ++i) {
    const TimeStamp stamp = 10 * (i + 1);
    frames.push_back(makeFrame(camera, stamp, makePixels(sizes[i])));
    // the second track sees every other frame
    first.observations.emplace_back(stamp, 1, -1, "test_camera");
    if (i % 2) {
      second.observations.emplace_back(stamp, 1, -1, "test_camera");
    }
  }

  for (const size_t max_images : {1, 3, 20}) {
    TrackLogger::Config config;
    config.max_images_per_object = max_images;
    const auto unbounded = logWithBuffer(config, 100, frames, {first, second});
    for (const size_t buffer_size : {1, 2, 5, 11}) {
      size_t num_evicted = 0;
      EXPECT_EQ(logWithBuffer(config, buffer_size, frames, {first, second}, &num_evicted),
                unbounded)
          << "max_images " << max_images << ", buffer " << buffer_size;
      EXPECT_GT(num_evicted, 0u);
    }
  }
}

TEST(TrackLogger, AbsorbedTrackSnapshotsMoveToAbsorber) {
  const auto camera = makeCamera();
  const auto best = makeFrame(camera, 10, makePixels(5));
  const auto other = makeFrame(camera, 20, makePixels(1));
  const auto unrelated = makeFrame(camera, 30, makePixels(1));
  Track absorbed;
  absorbed.id = 7;
  absorbed.first_seen = 10;
  absorbed.last_seen = 10;
  absorbed.observations.emplace_back(10, 1, -1, "test_camera");
  Track survivor;
  survivor.id = 4;
  survivor.first_seen = 5;
  survivor.last_seen = 20;
  survivor.observations.emplace_back(20, 1, -1, "test_camera");

  const auto root = makeTempDir();
  TrackLogger::Config config;
  config.log_path = root;
  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  buffer.storeData(other);
  {
    TrackLogger logger(config);
    logger.snapshot(*best, {absorbed, survivor});  // dropped while track 7 is live
    EXPECT_EQ(logger.numEvicted(true), 1u);
    survivor.absorb(std::move(absorbed));
    // the next dropped frame moves the snapshots of track 7 to track 4
    logger.snapshot(*unrelated, {survivor});
    EXPECT_EQ(logger.numEvicted(true), 1u);
    logger.log(survivor, buffer, true, logger.takeEvicted(survivor));
    logger.flush();
    EXPECT_EQ(logger.numEvicted(), 0u);
  }

  const auto json = readFile(root / "4" / "track.json");
  EXPECT_NE(json.find("\"absorbed_track_ids\": [7]"), std::string::npos) << json;
  EXPECT_NE(json.find("{\"k\": 0, \"timestamp_ns\": 10"), std::string::npos) << json;
  EXPECT_NE(json.find("\"score\": 5, \"kept\": true, \"color\": \"obs_0_color.jpg\""),
            std::string::npos)
      << json;
  EXPECT_NE(json.find("{\"k\": 1, \"timestamp_ns\": 20"), std::string::npos) << json;
  EXPECT_TRUE(std::filesystem::exists(root / "4" / "obs_0_color.jpg"));
  EXPECT_FALSE(std::filesystem::exists(root / "7"));
  std::filesystem::remove_all(root);
}

TEST(TrackLogger, TakesSnapshotsOfAbsorbedTracksWithBestImagesOnly) {
  const auto camera = makeCamera();
  Track first, second;
  first.id = 1;
  first.observations.emplace_back(10, 1, -1, "test_camera");
  second.id = 2;
  second.observations.emplace_back(20, 1, -1, "test_camera");

  TrackLogger::Config config;
  config.log_path = makeTempDir();
  TrackLogger logger(config);
  logger.snapshot(*makeFrame(camera, 10, makePixels(2)), {first, second});
  logger.snapshot(*makeFrame(camera, 20, makePixels(3)), {first, second});
  EXPECT_EQ(logger.numEvicted(true), 2u);  // each track keeps its best image

  // absorbed after the last dropped frame: taking the absorber's snapshots includes track 2's
  first.absorb(std::move(second));
  const auto evicted = logger.takeEvicted(first);
  ASSERT_EQ(evicted.size(), 2u);
  EXPECT_EQ(logger.numEvicted(), 0u);
  EXPECT_EQ(logger.evictedBytes(), 0u);
  size_t with_images = 0;
  for (const auto& observation : evicted) {
    with_images += observation.images != nullptr;
    EXPECT_EQ(observation.images != nullptr, observation.stamp == 20) << observation.stamp;
  }
  EXPECT_EQ(with_images, 1u);
  std::filesystem::remove_all(config.log_path);
}

TEST(TrackLogger, DropsSnapshotsOfDeletedTracks) {
  const auto camera = makeCamera();
  Track track;
  track.id = 5;
  track.observations.emplace_back(10, 1, -1, "test_camera");
  Track unrelated;
  unrelated.id = 6;

  TrackLogger::Config config;
  config.log_path = makeTempDir();
  TrackLogger logger(config);
  logger.snapshot(*makeFrame(camera, 10, makePixels(2)), {track, unrelated});
  EXPECT_EQ(logger.numEvicted(), 1u);
  EXPECT_GT(logger.evictedBytes(), 0u);

  // track 5 is gone (deleted without being logged) when the next frame is dropped
  logger.snapshot(*makeFrame(camera, 20, makePixels(2)), {unrelated});
  EXPECT_EQ(logger.numEvicted(), 0u);
  EXPECT_EQ(logger.evictedBytes(), 0u);
  EXPECT_GT(logger.peakEvictedBytes(), 0u);
  EXPECT_TRUE(logger.takeEvicted(track).empty());
  std::filesystem::remove_all(config.log_path);
}

TEST(TrackLogger, NoSnapshotsWhenDisabledOrNothingIsDropped) {
  const auto camera = makeCamera();
  Track track;
  track.id = 5;
  track.observations.emplace_back(10, 1, -1, "test_camera");

  TrackLogger disabled(TrackLogger::Config{});
  disabled.snapshot(*makeFrame(camera, 10, makePixels(2)), {track});
  EXPECT_EQ(disabled.numEvicted(), 0u);
  EXPECT_TRUE(disabled.takeEvicted(track).empty());

  // a buffer that never drops a frame never needs a snapshot
  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  for (TimeStamp stamp = 10; stamp <= 50; stamp += 10) {
    EXPECT_EQ(buffer.storeData(makeFrame(camera, stamp, makePixels(1))), nullptr);
  }
}

TEST(TrackLogger, WorkerPoolPassesSnapshotsToTheLogger) {
  const auto root = makeTempDir();
  TrackLogger::Config config;
  config.log_path = root;
  auto logger = std::make_shared<TrackLogger>(config);

  Track track;
  track.id = 9;
  track.observations.emplace_back(10, 1, -1, "test_camera");
  logger->snapshot(*makeFrame(makeCamera(), 10, makePixels(2)), {track});
  {
    ObjectWorkerPool pool(ObjectWorkerPool::Config{}, nullptr, logger);
    pool.submit(0, track, FrameDataBuffer(FrameDataBuffer::Config{}));
  }
  logger->flush();
  EXPECT_EQ(logger->numEvicted(), 0u);

  // dropped tracks stay metadata only
  const auto json = readFile(root / "9" / "track.json");
  EXPECT_NE(json.find("{\"k\": 0, \"timestamp_ns\": 10"), std::string::npos) << json;
  EXPECT_NE(json.find("\"mask_rle\": \"[0,2,10]\""), std::string::npos) << json;
  EXPECT_EQ(json.find("\"kept\": true"), std::string::npos) << json;
  std::filesystem::remove_all(root);
}

}  // namespace khronos
