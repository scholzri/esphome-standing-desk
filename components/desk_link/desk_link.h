#pragma once
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "frame_codec.h"

namespace esphome {
namespace desk_link {

class DeskLink : public Component {
 public:
  void set_keypad_uart(uart::UARTComponent *u) { this->keypad_uart_ = u; }
  void set_controlbox_uart(uart::UARTComponent *u) { this->controlbox_uart_ = u; }
  void set_capture_port(uint16_t p) { this->capture_port_ = p; }
  void set_stall_guard_ms(uint32_t ms) { this->stall_guard_ms_ = ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Sends `data` to the control box `count` times, `interval_ms` apart, for
  // genuine one-shot frames the keypad itself would only ever send once, such
  // as preset recall. This no longer suppresses the keypad stream: the byte
  // interleaving is left to hardware arbitration on the shared UART, exactly
  // as two independent writers would behave. Anything longer than
  // MAX_FRAME_LEN is rejected, which makes an over-long injection impossible
  // by construction rather than by convention.
  void inject(const uint8_t *data, uint8_t len, uint16_t count, uint16_t interval_ms);

  // Rewrites the command byte of every subsequent idle keypad frame
  // (AA FF xx 40 ..) to `cmdbyte` for `duration_ms`, as if a key were
  // physically held. duration_ms == 0 cancels immediately (RELEASE).
  // duration_ms is capped internally so a dropped capture connection cannot
  // leave the desk driving indefinitely.
  void set_hold(uint8_t cmdbyte, uint32_t duration_ms);

 protected:
  void handle_keypad_byte_(uint8_t b);
  void handle_box_byte_(uint8_t b);
  void emit_frame_(ctb::Direction dir, const ctb::Frame &f);
  void flush_hold_(const char *reason);
  void service_hold_expiry_();
  void service_injection_();
  void service_capture_();
  void handle_command_(char *line);
  void capture_write_(const char *s, size_t len);

  uart::UARTComponent *keypad_uart_{nullptr};
  uart::UARTComponent *controlbox_uart_{nullptr};

  ctb::FrameCodec keypad_codec_{ctb::Direction::KEYPAD_TO_BOX};
  ctb::FrameCodec box_codec_{ctb::Direction::BOX_TO_KEYPAD};

  uint16_t capture_port_{6789};
  int listen_fd_{-1};
  int client_fd_{-1};
  char cmd_buf_[192];
  uint8_t cmd_len_{0};

  // How long held keypad bytes may sit unforwarded before the stall guard
  // (see loop()) flushes them unmodified. Defaults to 30 ms in __init__.py;
  // see the comment there for why 30 ms is the chosen value.
  uint32_t stall_guard_ms_{30};

  uint8_t inject_buf_[ctb::MAX_FRAME_LEN];
  uint8_t inject_len_{0};
  uint16_t inject_remaining_{0};
  uint16_t inject_interval_ms_{38};
  uint32_t inject_next_ms_{0};

  // Keypad-to-box store-and-forward. Every keypad byte -- there is no
  // byte-immediate path any more -- is appended to hold_buf_ and fed to
  // keypad_codec_ in lockstep, so hold_buf_[0 .. hold_len_) always holds
  // exactly the bytes of the frame keypad_codec_ is currently assembling
  // (or has just completed), whether or not any of it has already been
  // written out.
  //
  // hold_len_ alone cannot tell you whether the codec is mid-frame: the
  // stall guard (and the full-buffer guard) can write held bytes out to the
  // control box while the codec is still genuinely mid-frame, and neither
  // may reset hold_len_ to 0 at that point without losing track of the
  // frame's already-buffered bytes. So a second counter, hold_sent_, tracks
  // how many bytes of the *current* frame have already been written to the
  // control box. hold_sent_ <= hold_len_ always. On frame close:
  //   - hold_sent_ == 0 means nothing of this frame went out yet, so the
  //     whole frame is still eligible for rewrite.
  //   - hold_sent_ > 0 means a prefix already left the building verbatim;
  //     only the unsent tail hold_buf_[hold_sent_ .. hold_len_) is forwarded,
  //     never rewritten, since bytes already on the wire can't be recalled.
  // Both counters reset to 0 only once a frame is fully accounted for
  // (closed-and-forwarded) or the full-buffer guard forces a hard reset.
  uint8_t hold_buf_[ctb::MAX_FRAME_LEN];
  uint8_t hold_len_{0};
  uint8_t hold_sent_{0};
  uint32_t hold_start_ms_{0};

  // Command-byte override ("hold"), applied only to idle (cmd 0x40) frames.
  bool override_active_{false};
  uint8_t override_cmd_{0};
  uint32_t override_deadline_ms_{0};

  uint32_t seq_{0};
  uint32_t drops_{0};

  // Counts of flush_hold_() calls by cause, reported in STAT so a workstation
  // script can confirm from outside how often the stall guard is firing --
  // it should be rare at the default 30 ms, and every firing means some
  // frame's tail was forwarded unrewritten (see flush_hold_ and the
  // stall-guard check in loop()).
  uint32_t stall_flushes_{0};
  uint32_t full_buffer_flushes_{0};
};

}  // namespace desk_link
}  // namespace esphome
