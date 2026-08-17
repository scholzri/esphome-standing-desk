#include "frame_codec.h"
#include "ctb_crc.h"
#include <string.h>

namespace ctb {

uint8_t expected_frame_len(Direction dir, uint8_t type) {
  if (dir == Direction::KEYPAD_TO_BOX) {
    switch (type) {
      case 0xFF: return 6;   // button and movement commands
      case 0xF4: return 4;
      case 0xF2: return 28;  // configuration write
      default: return 0;
    }
  }
  switch (type) {
    case 0xFF: return 7;     // height report
    default: return 0;
  }
}

void FrameCodec::reset_() {
  len_ = 0;
  expected_ = 0;
  in_frame_ = false;
  scanning_ = false;
}

bool FrameCodec::emit_(Frame *out, bool length_known, bool crc_valid) {
  memcpy(out->bytes, buf_, len_);
  out->len = len_;
  out->crc_valid = crc_valid;
  out->length_known = length_known;
  reset_();
  return true;
}

bool FrameCodec::put(uint8_t b, Frame *out) {
  if (!in_frame_) {
    if (b != SYNC_BYTE) {
      resyncs_++;
      return false;
    }
    in_frame_ = true;
    len_ = 0;
  }

  if (len_ >= MAX_FRAME_LEN) {
    // Gave up: no CRC-valid boundary within the largest frame we allow.
    crc_errors_++;
    reset_();
    // Reprocess this byte as a potential new sync.
    return put(b, out);
  }

  buf_[len_++] = b;

  // The type byte determines the tabulated length.
  if (len_ == 2) {
    expected_ = expected_frame_len(dir_, buf_[1]);
    scanning_ = (expected_ == 0);
  }

  if (!scanning_ && len_ == expected_) {
    if (ctb::crc_check(buf_, len_))
      return emit_(out, true, true);
    // The table said this length but the CRC disagrees. Rather than accept a
    // bad frame, keep accumulating and look for a real boundary. This makes
    // the codec tolerant of an incomplete length table during discovery.
    crc_errors_++;
    scanning_ = true;
    return false;
  }

  if (scanning_ && len_ >= MIN_FRAME_LEN && ctb::crc_check(buf_, len_))
    return emit_(out, false, true);

  return false;
}

}  // namespace ctb
