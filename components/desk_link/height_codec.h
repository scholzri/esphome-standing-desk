#pragma once
#include "frame_codec.h"

namespace ctb {

// The box's blank-display frame, sent while the desk is asleep. All three
// digit bytes are 0x14, which is digit value 10 in every position. It is not a
// height: the shipped standing_desk_height decoder accepts it and reports
// 212.0 cm.
inline bool is_sentinel_frame(const Frame &f) {
  return f.len == 7 && f.bytes[0] == SYNC_BYTE && f.bytes[1] == 0xFF &&
         f.bytes[2] == 0x14 && f.bytes[3] == 0x14 && f.bytes[4] == 0x14;
}

// Decodes a box-to-keypad height report. Returns a negative value when the
// frame is not a height report.
//
// The payload is literally the three characters on the control box display:
//   below 100 cm: b2 = tens, b1 = units + 10 (the +10 lights the decimal
//                 point), b0 = tenths            -> 90.8
//   at/above 100: b2 = hundreds, b1 = tens, b0 = units, no decimal point,
//                 so resolution is a whole centimetre  -> 121
inline float decode_height_frame(const Frame &f) {
  if (f.len != 7 || f.bytes[0] != SYNC_BYTE || f.bytes[1] != 0xFF)
    return -1.0f;
  if (is_sentinel_frame(f))
    return -1.0f;

  // Honour the codec's CRC verdict. A corrupted frame that still looks
  // digit-shaped would otherwise decode to a plausible, wrong height, and
  // acting on a wrong height moves the desk to the wrong place.
  if (!f.crc_valid)
    return -1.0f;

  const uint8_t b0 = f.bytes[2], b1 = f.bytes[3], b2 = f.bytes[4];
  if (b0 > 9 || b2 > 9 || b1 > 19)
    return -1.0f;   // not digits; not a height report

  if (b1 >= 10)
    return (b2 * 100 + (b1 - 10) * 10 + b0) / 10.0f;
  return (float) (b2 * 100 + b1 * 10 + b0);
}

}  // namespace ctb
