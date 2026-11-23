#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include <gtest/gtest.h>
#include <hydra/input/camera.h>
#include <hydra/input/sensor_extrinsics.h>
#include <hydra/utils/temp_path.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "khronos/active_window/object_extraction/object_worker_pool.h"
#include "khronos/active_window/track_logger.h"

namespace khronos {
namespace {

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
                         const Pixels& pixels) {
  hydra::InputData input(camera);
  input.timestamp_ns = stamp;
  input.world_T_body = Eigen::Isometry3d::Identity();
  input.world_T_body.translation() << 1.0, 2.0, 3.0;
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

using Json = nlohmann::json;

Json readJson(const std::filesystem::path& path) { return Json::parse(readFile(path)); }

// The keyframe archives' pose layout: translation in metres and a wxyz quaternion
Json poseJson(const Eigen::Vector3d& p, const Eigen::Quaterniond& q = {1, 0, 0, 0}) {
  return {{"translation_m", {p.x(), p.y(), p.z()}},
          {"rotation_wxyz", {q.w(), q.x(), q.y(), q.z()}}};
}

size_t numKept(const Json& track) {
  size_t result = 0;
  for (const auto& observation : track["observations"]) {
    result += observation["kept"].get<bool>();
  }

  return result;
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
                                                 const Tracks& tracks) {
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  const auto& root = temp.path;
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

    for (const auto& track : tracks) {
      logger.log(track, buffer, true, logger.takeEvicted(track));
    }

    logger.flush();
  }

  return readTree(root);
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

TEST(TrackLogger, WritesTrackAndKeptImages) {
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  const auto& root = temp.path;
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
  const auto text = readFile(track_dir / "track.json");
  EXPECT_EQ(text.rfind("{\n  \"version\": 1,", 0), 0u) << text;  // version first
  const auto json = Json::parse(text);
  EXPECT_EQ(json["robot_id"], 0);
  EXPECT_EQ(json["track_id"], 12);
  EXPECT_EQ(json["absorbed_track_ids"], Json({7, 9}));
  EXPECT_EQ(json["extracted"], true);
  EXPECT_EQ(json["semantic_label"], 3);
  ASSERT_EQ(json["observations"].size(), 2u) << text;
  // non-kept observation: metadata only plus RLE mask
  const Json expected_first{{"k", 0},
                            {"timestamp_ns", 10},
                            {"world_T_body", poseJson({1.0, 2.0, 3.0})},
                            {"sensor", "test_camera"},
                            {"image_size", {4, 3}},
                            {"bbox_2d", {1, 1, 1, 1}},
                            {"mask_rle", "[5,1,6]"},
                            {"score", 1.0},
                            {"kept", false},
                            {"color", nullptr},
                            {"depth", nullptr},
                            {"mask", nullptr}};
  EXPECT_EQ(json["observations"][0], expected_first) << text;
  const auto& second = json["observations"][1];
  EXPECT_EQ(second["k"], 1);
  EXPECT_EQ(second["timestamp_ns"], 20);
  EXPECT_EQ(second["bbox_2d"], Json({1, 0, 2, 1}));
  EXPECT_EQ(second["mask_rle"], nullptr);
  EXPECT_EQ(second["score"], 3.0);
  EXPECT_EQ(second["kept"], true);
  EXPECT_EQ(second["color"], "obs_1_color.jpg");
  EXPECT_EQ(second["depth"], "obs_1_depth.png");
  EXPECT_EQ(second["mask"], "obs_1_mask.png");

  // the sensor's record is written once, as in the keyframe archives' metadata.yaml
  const auto sensors = YAML::LoadFile((root / "tracks" / "sensors.yaml").string());
  EXPECT_EQ(sensors["version"].as<int>(), 1);
  ASSERT_EQ(sensors["sensors"].size(), 1u);
  const auto sensor = sensors["sensors"][0];
  EXPECT_EQ(sensor["name"].as<std::string>(), "test_camera");
  EXPECT_EQ(sensor["config"]["type"].as<std::string>(), "camera");
  EXPECT_EQ(sensor["config"]["fx"].as<double>(), 10.0);
  EXPECT_EQ(sensor["config"]["cy"].as<double>(), 1.5);
  EXPECT_EQ(sensor["config"]["width"].as<int>(), 4);
  EXPECT_EQ(sensor["config"]["extrinsics"]["type"].as<std::string>(), "param");

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
}

TEST(TrackLogger, LogsFailedExtraction) {
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  const auto& root = temp.path;
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
    const auto json = readJson(root / id / "track.json");
    EXPECT_EQ(json["extracted"], false);
    EXPECT_EQ(json["semantic_label"], nullptr);
    // dropped tracks are metadata only: no kept observation and no image files
    EXPECT_EQ(json["observations"].size(), 1u);
    EXPECT_EQ(numKept(json), 0u);
    for (const auto& entry : std::filesystem::directory_iterator(root / id)) {
      EXPECT_EQ(entry.path().filename(), "track.json") << entry.path();
    }
  }
}

TEST(TrackLogger, KeepsImagesOfDroppedTracksWhenConfigured) {
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  const auto& root = temp.path;
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

  const auto json = readJson(root / "6" / "track.json");
  EXPECT_EQ(json["extracted"], false);
  EXPECT_EQ(numKept(json), 1u);
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
  const auto evicted = logWithBuffer(config, 2, frames, {track});
  ASSERT_TRUE(evicted.count("3/track.json"));
  const auto json = Json::parse(evicted.at("3/track.json"));
  ASSERT_EQ(json["observations"].size(), 3u);
  const auto& best = json["observations"][0];
  EXPECT_EQ(best["timestamp_ns"], 10);
  EXPECT_EQ(best["bbox_2d"], Json({0, 0, 2, 0}));
  EXPECT_EQ(best["mask_rle"], nullptr);
  EXPECT_EQ(best["score"], 3.0);
  EXPECT_EQ(best["kept"], true);
  EXPECT_EQ(best["color"], "obs_0_color.jpg");
  EXPECT_EQ(best["depth"], "obs_0_depth.png");
  EXPECT_EQ(best["mask"], "obs_0_mask.png");
  EXPECT_EQ(json["observations"][2]["timestamp_ns"], 30);
  EXPECT_TRUE(evicted.count("sensors.yaml"));
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

  for (const auto& [max_images, buffer_size] :
       {std::pair{1u, 1u}, std::pair{3u, 10u}, std::pair{20u, 5u}}) {
    SCOPED_TRACE(::testing::Message() << max_images << " images, buffer " << buffer_size);
    TrackLogger::Config config;
    config.max_images_per_object = max_images;
    const auto unbounded = logWithBuffer(config, 100, frames, {first, second});
    for (const auto id : {1, 2}) {
      const auto path = std::to_string(id) + "/track.json";
      ASSERT_TRUE(unbounded.count(path));
      const auto observations = Json::parse(unbounded.at(path))["observations"];
      ASSERT_EQ(observations.size(), id == 1 ? 12u : 6u);
      std::vector<TimeStamp> kept;
      for (const auto& observation : observations) {
        if (observation["kept"].get<bool>()) {
          kept.push_back(observation["timestamp_ns"].get<TimeStamp>());
        }
      }

      std::vector<TimeStamp> expected;
      if (max_images == 1) {
        expected = {60};
      } else if (max_images == 3) {
        expected =
            id == 1 ? std::vector<TimeStamp>{60, 100, 110} : std::vector<TimeStamp>{20, 60, 100};
      } else {
        expected = id == 1
                       ? std::vector<TimeStamp>{10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120}
                       : std::vector<TimeStamp>{20, 40, 60, 80, 100, 120};
      }

      EXPECT_EQ(kept, expected);
    }

    EXPECT_EQ(logWithBuffer(config, buffer_size, frames, {first, second}), unbounded);
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

  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  const auto& root = temp.path;
  TrackLogger::Config config;
  config.log_path = root;
  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  buffer.storeData(other);
  {
    TrackLogger logger(config);
    logger.snapshot(*best, {absorbed, survivor});  // dropped while track 7 is live
    survivor.absorb(std::move(absorbed));
    // the next dropped frame moves the snapshots of track 7 to track 4
    logger.snapshot(*unrelated, {survivor});
    logger.log(survivor, buffer, true, logger.takeEvicted(survivor));
    logger.flush();
  }

  const auto json = readJson(root / "4" / "track.json");
  EXPECT_EQ(json["absorbed_track_ids"], Json({7}));
  ASSERT_EQ(json["observations"].size(), 2u);
  const auto& first = json["observations"][0];
  EXPECT_EQ(first["timestamp_ns"], 10);
  EXPECT_EQ(first["score"], 5.0);
  EXPECT_EQ(first["kept"], true);
  EXPECT_EQ(first["color"], "obs_0_color.jpg");
  EXPECT_EQ(json["observations"][1]["timestamp_ns"], 20);
  // the snapshot registered the sensor of the dropped frame
  EXPECT_TRUE(std::filesystem::exists(root / "sensors.yaml"));
  EXPECT_TRUE(std::filesystem::exists(root / "4" / "obs_0_color.jpg"));
  EXPECT_FALSE(std::filesystem::exists(root / "7"));
}

TEST(TrackLogger, TakesSnapshotsOfAbsorbedTracksWithBestImagesOnly) {
  const auto camera = makeCamera();
  Track first, second;
  first.id = 1;
  first.observations.emplace_back(10, 1, -1, "test_camera");
  second.id = 2;
  second.observations.emplace_back(20, 1, -1, "test_camera");

  TrackLogger::Config config;
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  config.log_path = temp.path;
  TrackLogger logger(config);
  logger.snapshot(*makeFrame(camera, 10, makePixels(2)), {first, second});
  logger.snapshot(*makeFrame(camera, 20, makePixels(3)), {first, second});

  // absorbed after the last dropped frame: taking the absorber's snapshots includes track 2's
  first.absorb(std::move(second));
  const auto evicted = logger.takeEvicted(first);
  ASSERT_EQ(evicted.size(), 2u);
  EXPECT_TRUE(logger.takeEvicted(first).empty());
  size_t with_images = 0;
  for (const auto& observation : evicted) {
    with_images += observation.images != nullptr;
    EXPECT_EQ(observation.images != nullptr, observation.stamp == 20) << observation.stamp;
  }

  EXPECT_EQ(with_images, 1u);
}

TEST(TrackLogger, DropsSnapshotsOfDeletedTracks) {
  const auto camera = makeCamera();
  Track track;
  track.id = 5;
  track.observations.emplace_back(10, 1, -1, "test_camera");
  Track unrelated;
  unrelated.id = 6;
  unrelated.observations.emplace_back(10, 1, -1, "test_camera");

  TrackLogger::Config config;
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  config.log_path = temp.path;
  TrackLogger logger(config);
  logger.snapshot(*makeFrame(camera, 10, makePixels(2)), {track, unrelated});

  // track 5 is gone (deleted without being logged) when the next frame is dropped
  logger.snapshot(*makeFrame(camera, 20, makePixels(2)), {unrelated});
  EXPECT_TRUE(logger.takeEvicted(track).empty());
  const auto kept = logger.takeEvicted(unrelated);
  ASSERT_EQ(kept.size(), 1u);
  EXPECT_EQ(kept.front().stamp, 10u);
  EXPECT_TRUE(kept.front().images);
}

TEST(TrackLogger, DisabledWithoutPath) {
  const auto camera = makeCamera();
  Track track;
  track.id = 5;
  track.observations.emplace_back(10, 1, -1, "test_camera");

  TrackLogger disabled(TrackLogger::Config{});
  EXPECT_FALSE(disabled.enabled());
  disabled.snapshot(*makeFrame(camera, 10, makePixels(2)), {track});
  EXPECT_TRUE(disabled.takeEvicted(track).empty());
  disabled.log(track, FrameDataBuffer(FrameDataBuffer::Config{}), true);
  disabled.flush();
}

TEST(TrackLogger, WorkerPoolPassesSnapshotsToTheLogger) {
  const auto temp = hydra::utils::TempDirectory::system_default("khronos-tracks-", true);
  const auto& root = temp.path;
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

  // dropped tracks stay metadata only
  const auto json = readJson(root / "9" / "track.json");
  ASSERT_EQ(json["observations"].size(), 1u);
  EXPECT_EQ(json["observations"][0]["timestamp_ns"], 10);
  EXPECT_EQ(json["observations"][0]["mask_rle"], "[0,2,10]");
  EXPECT_EQ(numKept(json), 0u);
  logger.reset();
}

}  // namespace khronos
