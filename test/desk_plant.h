#pragma once
#include "../components/desk_link/motion_controller.h"
#include <cmath>
#include <cstdint>

// A simulated desk built from constants measured on the real hardware during
// Plan 1. Fixtures backing them are in test/fixtures/.
class DeskPlant {
 public:
  // Measured: ~3.5 cm/s up, ~3.9 cm/s down (gravity assisted).
  static constexpr float SPEED_UP = 3.5f;
  static constexpr float SPEED_DOWN = 3.9f;
  // Measured: soft-start ramp of about 1.2 s to reach top speed.
  static constexpr float RAMP_S = 1.2f;
  // Measured: 68 ms from command to motion for a raw button press.
  static constexpr uint32_t LATENCY_MS = 68;
  // Measured coast: 1.9 cm at 3.5 cm/s up, 0.4 cm at 3.9 cm/s down. Modelled
  // as a deceleration time so coast scales with the speed actually reached.
  static constexpr float DECEL_UP_S = 1.9f / 3.5f * 2.0f;
  static constexpr float DECEL_DOWN_S = 0.4f / 3.9f * 2.0f;

  explicit DeskPlant(float start_cm) : height_(start_cm) {}

  void command(ctb::MoveCommand c) {
    if (c != pending_) {
      pending_ = c;
      latency_left_ = LATENCY_MS;
    }
  }

  void tick(uint32_t dt_ms) {
    const float dt = dt_ms / 1000.0f;
    if (latency_left_ > 0) {
      latency_left_ = (latency_left_ > dt_ms) ? latency_left_ - dt_ms : 0;
    } else {
      active_ = pending_;
    }

    const float target_v = (active_ == ctb::MoveCommand::UP)     ? SPEED_UP
                           : (active_ == ctb::MoveCommand::DOWN) ? -SPEED_DOWN
                                                                 : 0.0f;
    // Accelerate on the soft-start ramp, decelerate on the measured, and
    // direction-specific, braking time.
    float rate;
    if (target_v == 0.0f)
      rate = (velocity_ > 0 ? SPEED_UP / DECEL_UP_S : SPEED_DOWN / DECEL_DOWN_S);
    else
      rate = (target_v > 0 ? SPEED_UP : SPEED_DOWN) / RAMP_S;

    const float step = rate * dt;
    if (velocity_ < target_v)
      velocity_ = std::fmin(target_v, velocity_ + step);
    else if (velocity_ > target_v)
      velocity_ = std::fmax(target_v, velocity_ - step);

    height_ += velocity_ * dt;
  }

  float true_height() const { return height_; }

  // What the control box would report: three display characters, so 0.1 cm
  // resolution below 100 cm and whole centimetres at or above it.
  float reported_height() const {
    if (height_ >= 100.0f)
      return std::round(height_);
    return std::round(height_ * 10.0f) / 10.0f;
  }

 private:
  float height_;
  float velocity_{0.0f};
  ctb::MoveCommand pending_{ctb::MoveCommand::NONE};
  ctb::MoveCommand active_{ctb::MoveCommand::NONE};
  uint32_t latency_left_{0};
};
