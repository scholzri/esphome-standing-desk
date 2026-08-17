#pragma once
#include <stddef.h>
#include <stdint.h>

namespace ctb {

// CRC-16 used by CTB control boxes on every frame type.
//
// Parameters were recovered by brute-force search against ten known-good
// keypad frames and are the unique solution across 262144 candidate
// configurations:
//
//   poly=0x6005  init=0xFFFF  xorout=0x0000  refin=true  refout=true
//
// The reflected form below uses the reversed polynomial 0xA006 and shifts
// right, which is equivalent and avoids per-byte bit reversal.
inline uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (uint16_t) ((crc >> 1) ^ 0xA006) : (uint16_t) (crc >> 1);
  }
  return crc;
}

// Appends the CRC after `payload_len` bytes, in transmission order: low byte
// first. `frame` must have room for payload_len + 2 bytes.
inline void crc_append(uint8_t *frame, size_t payload_len) {
  uint16_t crc = crc16(frame, payload_len);
  frame[payload_len] = (uint8_t) (crc & 0xFF);
  frame[payload_len + 1] = (uint8_t) (crc >> 8);
}

// Validates a complete frame whose last two bytes are the CRC.
inline bool crc_check(const uint8_t *frame, size_t total_len) {
  if (total_len < 3)
    return false;
  const size_t payload_len = total_len - 2;
  const uint16_t crc = crc16(frame, payload_len);
  return frame[payload_len] == (uint8_t) (crc & 0xFF) &&
         frame[payload_len + 1] == (uint8_t) (crc >> 8);
}

}  // namespace ctb
