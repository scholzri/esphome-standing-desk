#pragma once
#include <stddef.h>
#include <stdint.h>

namespace ctb {

enum class Direction : uint8_t {
  KEYPAD_TO_BOX = 0,
  BOX_TO_KEYPAD = 1,
};

static const uint8_t SYNC_BYTE = 0xAA;
static const uint8_t MAX_FRAME_LEN = 64;
// Shortest frame that can carry a CRC: sync, type, one payload byte, two CRC.
static const uint8_t MIN_FRAME_LEN = 4;

struct Frame {
  uint8_t bytes[MAX_FRAME_LEN];
  uint8_t len;
  // False when the trailing two bytes did not validate. Bad frames are still
  // surfaced during capture rather than dropped, so the protocol map can
  // record what actually appeared on the wire.
  bool crc_valid;
  // False when the frame was discovered by CRC-terminated scanning rather
  // than by the length table.
  bool length_known;
};

// Returns the total frame length for a direction and type, or 0 when unknown.
uint8_t expected_frame_len(Direction dir, uint8_t type);

// Assembles frames from a byte stream in one direction.
//
// Framing deliberately avoids timing. ESPHome reads the UART from a buffer,
// so inter-byte gaps at read time bear no relation to the wire. Known types
// use the length table; unknown types are framed by scanning for a CRC-valid
// termination, which was verified to find exactly one boundary on real
// 28-byte config frames.
class FrameCodec {
 public:
  explicit FrameCodec(Direction dir) : dir_(dir) {}

  // Feeds one byte. Returns true when *out has been filled with a frame.
  bool put(uint8_t b, Frame *out);

  uint32_t crc_errors() const { return crc_errors_; }
  uint32_t resyncs() const { return resyncs_; }

 private:
  void reset_();
  bool emit_(Frame *out, bool length_known, bool crc_valid);

  Direction dir_;
  uint8_t buf_[MAX_FRAME_LEN];
  uint8_t len_{0};
  uint8_t expected_{0};
  bool in_frame_{false};
  bool scanning_{false};
  uint32_t crc_errors_{0};
  uint32_t resyncs_{0};
};

}  // namespace ctb
