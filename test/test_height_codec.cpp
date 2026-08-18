#include "../components/desk_link/height_codec.h"
#include "harness.h"

namespace {

ctb::Frame frame(std::initializer_list<uint8_t> bytes) {
  ctb::Frame f{};
  f.len = 0;
  for (uint8_t b : bytes) f.bytes[f.len++] = b;
  f.crc_valid = true;
  f.length_known = true;
  return f;
}

static bool decode_close(float a, float b) {
  return a > b - 0.05f && a < b + 0.05f;
}

}  // namespace

TEST(decodes_heights_below_one_metre) {
  // Below 100 cm the display reads XX.X: b2 tens, b1 units + 10 for the
  // decimal point, b0 tenths.
  CHECK(decode_close(ctb::decode_height_frame(
      frame({0xAA, 0xFF, 0x01, 0x12, 0x06, 0x26, 0x97})), 68.1f));
  CHECK(decode_close(ctb::decode_height_frame(
      frame({0xAA, 0xFF, 0x02, 0x10, 0x06, 0xC1, 0x52})), 66.2f));
  CHECK(decode_close(ctb::decode_height_frame(
      frame({0xAA, 0xFF, 0x03, 0x10, 0x07, 0x0D, 0x62})), 76.3f));
}

TEST(decodes_heights_at_and_above_one_metre) {
  // At and above 100 cm the display reads XXX with no decimal point:
  // b2 hundreds, b1 tens, b0 units. Resolution drops to 1 cm.
  CHECK(decode_close(ctb::decode_height_frame(
      frame({0xAA, 0xFF, 0x00, 0x00, 0x01, 0x00, 0x00})), 100.0f));
  CHECK(decode_close(ctb::decode_height_frame(
      frame({0xAA, 0xFF, 0x07, 0x00, 0x01, 0x00, 0x00})), 107.0f));
  CHECK(decode_close(ctb::decode_height_frame(
      frame({0xAA, 0xFF, 0x01, 0x02, 0x01, 0x00, 0x00})), 121.0f));
}

TEST(identifies_the_sleep_sentinel) {
  // All three digit bytes 0x14 means a blank display: the desk is asleep.
  ctb::Frame s = frame({0xAA, 0xFF, 0x14, 0x14, 0x14, 0x54, 0x61});
  CHECK(ctb::is_sentinel_frame(s));
  // The shipped decoder turns this into 212.0 cm and publishes it. Ours must
  // refuse it outright.
  CHECK(ctb::decode_height_frame(s) < 0.0f);
}

TEST(rejects_frames_that_are_not_height_reports) {
  CHECK(ctb::decode_height_frame(
            frame({0xAA, 0xFF, 0x00, 0x50, 0x2B, 0x08})) < 0.0f);   // 6-byte button frame
  CHECK(!ctb::is_sentinel_frame(
      frame({0xAA, 0xFF, 0x01, 0x12, 0x06, 0x26, 0x97})));
}
