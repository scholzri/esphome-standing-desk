#include "../components/desk_link/frame_codec.h"
#include "harness.h"
#include <vector>

namespace {

// Feeds bytes and collects every frame the codec emits.
std::vector<ctb::Frame> feed(ctb::FrameCodec &codec,
                             const std::vector<uint8_t> &bytes) {
  std::vector<ctb::Frame> out;
  ctb::Frame f;
  for (uint8_t b : bytes)
    if (codec.put(b, &f))
      out.push_back(f);
  return out;
}

// Keypad -> box, six bytes: "move up".
const std::vector<uint8_t> UP = {0xAA, 0xFF, 0x00, 0x50, 0x2B, 0x08};
// Keypad -> box, six bytes: "recall preset 1".
const std::vector<uint8_t> PRESET1 = {0xAA, 0xFF, 0x00, 0x48, 0x2A, 0x3E};
// Box -> keypad, seven bytes: height 68.1 cm. Verified against the live desk.
const std::vector<uint8_t> HEIGHT_681 = {0xAA, 0xFF, 0x01, 0x12,
                                         0x06, 0x26, 0x97};
// Keypad -> box, twenty-eight bytes: "display cm".
const std::vector<uint8_t> DISPLAY_CM = {
    0xAA, 0xF2, 0x31, 0x00, 0xDC, 0x05, 0x01, 0x00, 0x05, 0x00,
    0x90, 0x02, 0x5D, 0x02, 0x38, 0x00, 0x05, 0x00, 0x08, 0x00,
    0x04, 0x00, 0x02, 0x00, 0x00, 0x00, 0x69, 0xED};

}  // namespace

TEST(length_table_is_direction_aware) {
  // Type 0xFF means six bytes from the keypad but seven from the box.
  CHECK_EQ((int) ctb::expected_frame_len(ctb::Direction::KEYPAD_TO_BOX, 0xFF), 6);
  CHECK_EQ((int) ctb::expected_frame_len(ctb::Direction::BOX_TO_KEYPAD, 0xFF), 7);
  CHECK_EQ((int) ctb::expected_frame_len(ctb::Direction::KEYPAD_TO_BOX, 0xF4), 4);
  CHECK_EQ((int) ctb::expected_frame_len(ctb::Direction::KEYPAD_TO_BOX, 0xF2), 28);
  // Zero means "not known", which triggers CRC-terminated scanning.
  CHECK_EQ((int) ctb::expected_frame_len(ctb::Direction::BOX_TO_KEYPAD, 0x77), 0);
}

TEST(emits_known_length_keypad_frame) {
  ctb::FrameCodec codec(ctb::Direction::KEYPAD_TO_BOX);
  auto frames = feed(codec, UP);
  CHECK_EQ(frames.size(), (size_t) 1);
  CHECK_EQ((int) frames[0].len, 6);
  CHECK(frames[0].crc_valid);
  CHECK(frames[0].length_known);
  CHECK_EQ((int) frames[0].bytes[3], 0x50);
}

TEST(emits_known_length_box_frame) {
  ctb::FrameCodec codec(ctb::Direction::BOX_TO_KEYPAD);
  auto frames = feed(codec, HEIGHT_681);
  CHECK_EQ(frames.size(), (size_t) 1);
  CHECK_EQ((int) frames[0].len, 7);
  CHECK(frames[0].crc_valid);
  CHECK(frames[0].length_known);
}

TEST(discards_leading_garbage_and_counts_resync) {
  ctb::FrameCodec codec(ctb::Direction::KEYPAD_TO_BOX);
  std::vector<uint8_t> stream = {0x11, 0x22, 0x33};
  stream.insert(stream.end(), UP.begin(), UP.end());
  auto frames = feed(codec, stream);
  CHECK_EQ(frames.size(), (size_t) 1);
  CHECK(frames[0].crc_valid);
  CHECK_EQ(codec.resyncs(), (uint32_t) 3);
}

TEST(handles_back_to_back_frames) {
  ctb::FrameCodec codec(ctb::Direction::KEYPAD_TO_BOX);
  std::vector<uint8_t> stream = UP;
  stream.insert(stream.end(), PRESET1.begin(), PRESET1.end());
  auto frames = feed(codec, stream);
  CHECK_EQ(frames.size(), (size_t) 2);
  CHECK_EQ((int) frames[0].bytes[3], 0x50);
  CHECK_EQ((int) frames[1].bytes[3], 0x48);
}

TEST(discovers_unknown_type_by_crc_termination) {
  // Pretend 0xF2 is unknown by feeding it in the direction it was not
  // tabulated for. The codec must still find the boundary via the CRC.
  ctb::FrameCodec codec(ctb::Direction::BOX_TO_KEYPAD);
  auto frames = feed(codec, DISPLAY_CM);
  CHECK_EQ(frames.size(), (size_t) 1);
  CHECK_EQ((int) frames[0].len, 28);
  CHECK(frames[0].crc_valid);
  CHECK(!frames[0].length_known);
}

TEST(bad_crc_on_known_length_falls_back_to_scanning) {
  // A frame whose tabulated length is wrong must not be silently accepted.
  // The codec keeps accumulating and looks for a CRC-valid boundary instead.
  ctb::FrameCodec codec(ctb::Direction::KEYPAD_TO_BOX);
  std::vector<uint8_t> corrupted = UP;
  corrupted[4] = 0x00;  // break the CRC
  auto frames = feed(codec, corrupted);
  CHECK_EQ(frames.size(), (size_t) 0);
  CHECK_EQ(codec.crc_errors(), (uint32_t) 1);
}

TEST(recovers_after_a_corrupted_frame) {
  ctb::FrameCodec codec(ctb::Direction::KEYPAD_TO_BOX);
  std::vector<uint8_t> stream = UP;
  stream[4] = 0x00;
  // MAX_FRAME_LEN of filler forces the codec to give up and resync.
  for (int i = 0; i < ctb::MAX_FRAME_LEN; i++)
    stream.push_back(0x00);
  stream.insert(stream.end(), PRESET1.begin(), PRESET1.end());
  auto frames = feed(codec, stream);
  bool found_preset = false;
  for (const auto &f : frames)
    if (f.len == 6 && f.crc_valid && f.bytes[3] == 0x48)
      found_preset = true;
  CHECK(found_preset);
}
