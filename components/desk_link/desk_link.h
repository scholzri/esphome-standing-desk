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

  uint8_t inject_buf_[ctb::MAX_FRAME_LEN];
  uint8_t inject_len_{0};
  uint16_t inject_remaining_{0};
  uint16_t inject_interval_ms_{38};
  uint32_t inject_next_ms_{0};

  // Keypad-to-box store-and-forward. Bytes are held here, in lockstep with
  // whatever keypad_codec_ is accumulating internally, until a frame closes
  // or the stall guard fires. Only ever holds bytes belonging to a frame that
  // is genuinely still in flight: a byte is forwarded immediately, never
  // held, whenever this buffer is empty and the byte is not a sync byte, so
  // this buffer never contains leading garbage. That invariant is what lets
  // handle_keypad_byte_() in the .cpp compare hold_len_ directly against a
  // completed frame's length to decide whether the whole frame is still
  // pending (safe to rewrite) or partly already sent (must forward as-is).
  uint8_t hold_buf_[ctb::MAX_FRAME_LEN];
  uint8_t hold_len_{0};
  uint32_t hold_start_ms_{0};

  // Command-byte override ("hold"), applied only to idle (cmd 0x40) frames.
  bool override_active_{false};
  uint8_t override_cmd_{0};
  uint32_t override_deadline_ms_{0};

  uint32_t seq_{0};
  uint32_t drops_{0};
};

}  // namespace desk_link
}  // namespace esphome
