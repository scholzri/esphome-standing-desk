#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace desk_keypad {

class DeskKeypad : public Component {
 public:
  void set_keypad_uart(uart::UARTComponent *uart) { keypad_uart_ = uart; }
  void set_controlbox_uart(uart::UARTComponent *uart) { controlbox_uart_ = uart; }
  void set_height_sensor(sensor::Sensor *sensor) { height_sensor_ = sensor; }
  void set_injection_interval(uint32_t interval) { injection_interval_ = interval; }

  void loop() override;
  void inject_message(const uint8_t* message, uint8_t length);
  void move_to_target(float target_height);
  void stop_movement() { moving_to_target_ = false; }

 private:
  uart::UARTComponent *keypad_uart_{nullptr}, *controlbox_uart_{nullptr};
  sensor::Sensor *height_sensor_{nullptr};
  uint8_t injected_message_[6], current_keypad_message_[6];
  uint8_t injected_message_length_{0}, current_keypad_message_length_{0};
  float target_height_{0};
  uint32_t last_injection_time_{0}, injection_interval_{100};
  uint8_t expected_message_length_{0}, injection_count_{0};
  bool injection_active_{false}, in_keypad_message_{false}, moving_to_target_{false};

  static const uint8_t MOVE_UP_MESSAGE[6], MOVE_DOWN_MESSAGE[6];
  struct MessageInfo { uint8_t type, length; };
  static const MessageInfo KEYPAD_MESSAGES[2];

  void handle_injection();
  void handle_uart_data();
  void handle_keypad_byte(uint8_t byte);
  void send_message_to_controlbox(const uint8_t* message, uint8_t length);
};

}  // namespace desk_keypad
}  // namespace esphome
