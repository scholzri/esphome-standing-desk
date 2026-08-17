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

  // Sends `data` to the control box `count` times, `interval_ms` apart.
  // Keypad traffic toward the box is suppressed while a burst is in flight so
  // the two streams cannot interleave mid-frame. Anything longer than
  // MAX_FRAME_LEN is rejected, which makes an over-long injection impossible
  // by construction rather than by convention.
  void inject(const uint8_t *data, uint8_t len, uint16_t count, uint16_t interval_ms);

 protected:
  void handle_byte_(ctb::Direction dir, uint8_t b);
  void emit_frame_(ctb::Direction dir, const ctb::Frame &f);
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
  bool suppress_keypad_{false};

  uint32_t seq_{0};
  uint32_t drops_{0};
};

}  // namespace desk_link
}  // namespace esphome
