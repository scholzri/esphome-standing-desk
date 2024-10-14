#include "desk_keypad.h"
#include "esphome/core/log.h"

namespace esphome {
namespace desk_keypad {

static const char *TAG = "desk_keypad";
static const uint8_t KEYPAD_MESSAGE_START = 0xAA;
static const uint8_t INJECTION_REPEAT_COUNT = 3;
static const uint8_t MESSAGE_MAX_LENGTH = 6;

const uint8_t DeskKeypad::MOVE_UP_MESSAGE[6] = {0xAA, 0xFF, 0x00, 0x50, 0x2B, 0x08};
const uint8_t DeskKeypad::MOVE_DOWN_MESSAGE[6] = {0xAA, 0xFF, 0x00, 0x60, 0x29, 0x64};
const DeskKeypad::MessageInfo DeskKeypad::KEYPAD_MESSAGES[2] = {{0xFF, 6}, {0xF4, 4}};

void DeskKeypad::loop() {
  handle_injection();
  handle_uart_data();

  if (moving_to_target_ && height_sensor_ != nullptr) {
    float current_height = height_sensor_->state;
    if (std::abs(current_height - target_height_) <= 0.1) {
      stop_movement();
    } else {
      inject_message(current_height < target_height_ ? MOVE_UP_MESSAGE : MOVE_DOWN_MESSAGE, 6);
    }
  }
}

void DeskKeypad::handle_injection() {
  if (injection_active_ && millis() - last_injection_time_ >= injection_interval_) {
    if (injection_count_ < INJECTION_REPEAT_COUNT) {
      send_message_to_controlbox(injected_message_, injected_message_length_);
      injection_count_++;
      last_injection_time_ = millis();
    } else {
      injection_active_ = false;
    }
  }
}

void DeskKeypad::handle_uart_data() {
  while (keypad_uart_->available()) {
    uint8_t byte;
    keypad_uart_->read_byte(&byte);
    if (!injection_active_) handle_keypad_byte(byte);
  }

  while (controlbox_uart_->available()) {
    uint8_t byte;
    controlbox_uart_->read_byte(&byte);
    keypad_uart_->write_byte(byte);
  }
}

void DeskKeypad::handle_keypad_byte(uint8_t byte) {
  if (byte == KEYPAD_MESSAGE_START && !in_keypad_message_) {
    in_keypad_message_ = true;
    current_keypad_message_length_ = 0;
  }

  if (in_keypad_message_) {
    if (current_keypad_message_length_ < MESSAGE_MAX_LENGTH) {
      current_keypad_message_[current_keypad_message_length_++] = byte;
    }

    if (current_keypad_message_length_ == 2) {
      for (const auto& info : KEYPAD_MESSAGES) {
        if (info.type == current_keypad_message_[1]) {
          expected_message_length_ = info.length;
          break;
        }
      }
      if (expected_message_length_ == 0) {
        in_keypad_message_ = false;
        current_keypad_message_length_ = 0;
      }
    }

    if (current_keypad_message_length_ == expected_message_length_) {
      send_message_to_controlbox(current_keypad_message_, current_keypad_message_length_);
      in_keypad_message_ = false;
      current_keypad_message_length_ = 0;
    }
  }
}

void DeskKeypad::inject_message(const uint8_t* message, uint8_t length) {
  memcpy(injected_message_, message, length);
  injected_message_length_ = length;
  injection_active_ = true;
  injection_count_ = 0;
  last_injection_time_ = millis();
}

void DeskKeypad::send_message_to_controlbox(const uint8_t* message, uint8_t length) {
  controlbox_uart_->write_array(message, length);
}

void DeskKeypad::move_to_target(float target_height) {
  target_height_ = target_height;
  moving_to_target_ = true;
}

}  // namespace desk_keypad
}  // namespace esphome
