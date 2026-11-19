#include <cmath>
#include <optional>
#include <vector>

#include <gtest/gtest.h>
#include <khronos/active_window/change_detection/active_window_change_detector.h>
#include <khronos/active_window/data/track.h>

namespace khronos {
namespace {

using Detector = ActiveWindowChangeDetector;
using Presence = Detector::PresenceMeasurement;

constexpr float kVoxelSize = 0.1f;
constexpr TimeStamp kNow = 100'000'000'000;  // 100 s
constexpr TimeStamp kSecond = 1'000'000'000;

// Voxel center at integer voxel coordinates (x, 0, 0).
Point center(int x) { return Point((x + 0.5f) * kVoxelSize, 0.05f, 0.05f); }

TrackingVoxel& voxelAt(TrackingLayer& layer, const Point& point) {
  layer.allocateBlockPtr(layer.getBlockIndex(point));
  return *layer.getVoxelPtr(point);
}

// Presence with the default thresholds: observed within 0.5 s, occupied within 1 s, half observed.
Presence measure(const std::vector<Point>& footprint, const TrackingLayer& layer) {
  return Detector::measurePresence(footprint, layer, kNow, 0.5f, 0.5f, 1.0f);
}

TEST(AddedAbsence, PresenceCountsObservedAndOccupied) {
  TrackingLayer layer(kVoxelSize, 16);
  // 0: observed now, occupied now. 1: observed now, occupied 5 s ago (now free).
  // 2: observed now, never occupied. 3: observed 2 s ago (stale). 4: unallocated.
  auto& occupied = voxelAt(layer, center(0));
  occupied.last_observed = kNow;
  occupied.last_occupied = kNow;
  auto& freed = voxelAt(layer, center(1));
  freed.last_observed = kNow;
  freed.last_occupied = kNow - 5 * kSecond;
  auto& never = voxelAt(layer, center(2));
  never.last_observed = kNow;
  never.last_occupied = 0;
  auto& stale = voxelAt(layer, center(3));
  stale.last_observed = kNow - 2 * kSecond;
  stale.last_occupied = kNow - 2 * kSecond;
  const Point unallocated(100.0f, 100.0f, 100.0f);

  std::vector<Point> present, absent;
  const auto p = Detector::measurePresence(
      {center(0), center(1), center(2), center(3), unallocated},
      layer,
      kNow,
      0.5f,
      0.5f,
      1.0f,
      &present,
      &absent);
  EXPECT_EQ(p.num_voxels, 5);
  EXPECT_EQ(p.num_observed, 3);
  EXPECT_EQ(p.num_occupied, 1);
  EXPECT_TRUE(p.valid);  // 3 / 5 >= 0.5
  EXPECT_NEAR(p.presence, 1.0f / 3.0f, 1e-6);
  EXPECT_EQ(present.size(), 1u);
  EXPECT_EQ(absent.size(), 2u);
}

TEST(AddedAbsence, PresenceInvalidWhenTooLittleObserved) {
  TrackingLayer layer(kVoxelSize, 16);
  auto& observed = voxelAt(layer, center(0));
  observed.last_observed = kNow;
  observed.last_occupied = 0;
  for (int x = 1; x < 4; ++x) {
    voxelAt(layer, center(x));  // allocated, never observed (last_observed = 0)
  }
  const auto p = measure({center(0), center(1), center(2), center(3)}, layer);
  EXPECT_EQ(p.num_observed, 1);
  EXPECT_FALSE(p.valid);  // 1 / 4 < 0.5

  EXPECT_FALSE(measure({}, layer).valid);
}

TEST(AddedAbsence, OccupiedWithinBufferCountsAsPresent) {
  TrackingLayer layer(kVoxelSize, 16);
  auto& voxel = voxelAt(layer, center(0));
  voxel.last_observed = kNow;
  voxel.last_occupied = kNow - kSecond / 2;  // 0.5 s ago, inside the 1 s buffer
  EXPECT_FLOAT_EQ(measure({center(0)}, layer).presence, 1.0f);
  voxel.last_occupied = kNow - 2 * kSecond;
  EXPECT_FLOAT_EQ(measure({center(0)}, layer).presence, 0.0f);
}

TEST(AddedAbsence, CombineMeasurement) {
  Presence valid;
  valid.valid = true;
  valid.presence = 0.25f;
  const Presence invalid;

  // novelty + presence: n * p
  EXPECT_FLOAT_EQ(*Detector::combineAddedMeasurement(0.8f, 0.5f, valid), 0.2f);
  // novelty only: n
  EXPECT_FLOAT_EQ(*Detector::combineAddedMeasurement(0.8f, 0.5f, invalid), 0.8f);
  // presence only: last novelty * p
  EXPECT_FLOAT_EQ(*Detector::combineAddedMeasurement(std::nullopt, 0.5f, valid), 0.125f);
  // neither: no update
  EXPECT_FALSE(Detector::combineAddedMeasurement(std::nullopt, 0.5f, invalid).has_value());
}

TEST(AddedAbsence, ObservedEmptyWithdrawsAddedObject) {
  // A long-reported added object (like ada track 68), alpha 0.2, threshold 0.8.
  Detector::ChangeFilter filter;
  filter.change_confidence = 1.0f;
  filter.num_frames_observed = 392;
  filter.updateGate(0.8f, 20, kNow);
  ASSERT_TRUE(filter.is_changed);

  Presence gone;
  gone.valid = true;
  gone.presence = 0.0f;
  int frames = 0;
  while (filter.is_changed && frames < 100) {
    const auto m = Detector::combineAddedMeasurement(std::nullopt, 1.0f, gone);
    ASSERT_TRUE(m.has_value());
    filter.update(*m, 0.2f, frames);
    filter.updateGate(0.8f, 20, kNow);
    ++frames;
  }
  EXPECT_FALSE(filter.is_changed);
  EXPECT_EQ(frames, 2);  // 1.0 -> 0.8 (still >= 0.8) -> 0.64
  EXPECT_TRUE(filter.everChanged());

  // Still there: presence 1 keeps it added.
  Detector::ChangeFilter kept = Detector::ChangeFilter();
  kept.change_confidence = 1.0f;
  kept.num_frames_observed = 392;
  Presence there;
  there.valid = true;
  there.presence = 1.0f;
  for (int i = 0; i < 50; ++i) {
    kept.update(*Detector::combineAddedMeasurement(std::nullopt, 1.0f, there), 0.2f, i);
  }
  kept.updateGate(0.8f, 20, kNow);
  EXPECT_TRUE(kept.is_changed);
}

TEST(AddedAbsence, TrackFootprintRequantizes) {
  Track track;
  // Two tracker voxels (0.05 m) inside the same 0.1 m map voxel, one in the next.
  track.accumulateVoxels({GlobalIndex(0, 0, 0), GlobalIndex(1, 0, 0), GlobalIndex(2, 0, 0)},
                         0.05f,
                         kNow,
                         1);
  const auto footprint = Detector::trackFootprint(track, kVoxelSize);
  ASSERT_EQ(footprint.size(), 2u);
  for (const auto& point : footprint) {
    EXPECT_NEAR(point.y(), 0.05f, 1e-6);
    EXPECT_TRUE(std::abs(point.x() - 0.05f) < 1e-6 || std::abs(point.x() - 0.15f) < 1e-6);
  }
}

}  // namespace
}  // namespace khronos
