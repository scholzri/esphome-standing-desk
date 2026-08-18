#include "motion_controller.h"

namespace ctb {

namespace {
// Minimum span the velocity window must cover before it is trusted. Below
// this, two adjacent samples a few milliseconds apart would produce a huge,
// noisy velocity estimate.
const uint32_t VEL_MIN_SPAN_MS = 300;
}  // namespace

void MotionController::configure(const MotionConfig &cfg) { cfg_ = cfg; }

bool MotionController::elapsed_at_least_(uint32_t now_ms, uint32_t since_ms,
                                          uint32_t duration_ms) const {
  // Unsigned subtraction wraps correctly across a millis() rollover: even if
  // now_ms < since_ms numerically, (now_ms - since_ms) yields the true
  // elapsed time modulo 2^32.
  return (uint32_t) (now_ms - since_ms) >= duration_ms;
}

float MotionController::remaining_cm_(float height_cm) const {
  // Positive while still approaching the target in the locked direction;
  // crosses zero and goes negative once the desk has reached or passed it.
  return (direction_ == MoveCommand::UP) ? (target_cm_ - height_cm)
                                          : (height_cm - target_cm_);
}

float MotionController::deadband_at(float height_cm) const {
  return (height_cm >= 100.0f) ? cfg_.deadband_coarse_cm : cfg_.deadband_cm;
}

void MotionController::reset_velocity_window_() {
  vel_count_ = 0;
  vel_head_ = 0;
  velocity_cm_s_ = 0.0f;
}

void MotionController::record_sample_(float height_cm, uint32_t now_ms) {
  vel_h_[vel_head_] = height_cm;
  vel_t_[vel_head_] = now_ms;
  vel_head_ = (uint8_t) ((vel_head_ + 1) % VEL_WINDOW);
  if (vel_count_ < VEL_WINDOW)
    vel_count_++;

  if (vel_count_ < 2) {
    velocity_cm_s_ = 0.0f;
    return;
  }

  // Oldest sample currently held is `vel_count_` slots behind the write head.
  uint8_t oldest = (uint8_t) ((vel_head_ + VEL_WINDOW - vel_count_) % VEL_WINDOW);
  uint32_t span_ms = (uint32_t) (now_ms - vel_t_[oldest]);
  if (span_ms < VEL_MIN_SPAN_MS) {
    // Not enough history yet to trust a rate: stay at zero rather than
    // dividing by a near-zero, noisy time span.
    velocity_cm_s_ = 0.0f;
    return;
  }

  float dh = height_cm - vel_h_[oldest];
  velocity_cm_s_ = dh / (span_ms / 1000.0f);
}

void MotionController::enter_fault_(FaultReason reason) {
  state_ = MoveState::FAULT;
  fault_ = reason;
  command_ = MoveCommand::NONE;
}

void MotionController::start_excursion_(float from_height_cm, uint32_t now_ms) {
  direction_ = (target_cm_ >= from_height_cm) ? MoveCommand::UP : MoveCommand::DOWN;
  command_ = direction_;
  state_ = MoveState::MOVING;

  reset_velocity_window_();
  stall_anchor_set_ = false;

  // Immediately process this height as the excursion's first sample, rather
  // than waiting for the next update(). With only one sample in the window,
  // velocity_cm_s_ is zero, so this exercises the same stop-rule path (and
  // its zero-velocity guard) that every later sample goes through: the
  // guard is what stops a fresh excursion from being declared "arrived"
  // before the desk has moved at all.
  process_moving_sample_(from_height_cm, now_ms);
}

void MotionController::process_moving_sample_(float height_cm, uint32_t now_ms) {
  record_sample_(height_cm, now_ms);

  // Stall detection: if the box has not made the minimum expected progress
  // within the stall window, the motor is not actually turning (jammed,
  // unplugged, or the link dropped commands).
  if (!stall_anchor_set_) {
    stall_anchor_set_ = true;
    stall_anchor_cm_ = height_cm;
    stall_anchor_ms_ = now_ms;
  } else if ((height_cm - stall_anchor_cm_) > cfg_.stall_min_delta_cm ||
             (stall_anchor_cm_ - height_cm) > cfg_.stall_min_delta_cm) {
    stall_anchor_cm_ = height_cm;
    stall_anchor_ms_ = now_ms;
  } else if (elapsed_at_least_(now_ms, stall_anchor_ms_, cfg_.stall_ms)) {
    enter_fault_(FaultReason::STALL);
    return;
  }

  // Feed-forward stop: the motor keeps coasting after the command is
  // dropped, so the command must be pulled the moment the remaining
  // distance is no bigger than the distance the desk will coast through at
  // its current speed. A zero velocity estimate (fewer than two samples, or
  // fewer than 300 ms between the oldest and newest) must never satisfy
  // this, or the move would be declared stopped before the desk starts.
  float remaining = remaining_cm_(height_cm);
  float coast_s = (direction_ == MoveCommand::UP) ? cfg_.coast_up_s : cfg_.coast_down_s;
  float coast_cm = coast_s * ((velocity_cm_s_ < 0) ? -velocity_cm_s_ : velocity_cm_s_);
  if (velocity_cm_s_ != 0.0f && remaining <= coast_cm) {
    command_ = MoveCommand::NONE;
    state_ = MoveState::SETTLING;
    settle_start_ms_ = now_ms;
  }
}

void MotionController::begin_move(float target_cm, uint32_t now_ms) {
  fault_ = FaultReason::NONE;
  target_cm_ = target_cm;

  if (target_cm < cfg_.min_height_cm || target_cm > cfg_.max_height_cm) {
    // Rejected outright, never clamped: a caller that asks for an
    // out-of-range height has a bug worth surfacing, not silently working
    // around.
    command_ = MoveCommand::NONE;
    state_ = MoveState::FAULT;
    fault_ = FaultReason::OUT_OF_RANGE;
    awaiting_first_update_ = false;
    return;
  }

  move_start_ms_ = now_ms;
  correction_used_ = false;
  command_ = MoveCommand::NONE;
  // Optimistically MOVING: direction is not locked yet (no height sample
  // exists), but callers are entitled to treat the move as under way from
  // begin_move() onward. If the very first update() turns out to have no
  // valid height, that update() call demotes this to WAKING itself.
  state_ = MoveState::MOVING;
  awaiting_first_update_ = true;
}

void MotionController::abort(uint32_t now_ms) {
  (void) now_ms;
  enter_fault_(FaultReason::ABORTED);
}

void MotionController::update(float height_cm, bool valid, uint32_t now_ms) {
  if (state_ == MoveState::DONE || state_ == MoveState::FAULT)
    return;   // terminal: nothing left to command

  // Global timeout applies across the whole move, regardless of which
  // sub-state (waking, moving, settling) it is currently in.
  if (state_ != MoveState::IDLE &&
      elapsed_at_least_(now_ms, move_start_ms_, cfg_.timeout_ms)) {
    enter_fault_(FaultReason::TIMEOUT);
    return;
  }

  if (awaiting_first_update_) {
    awaiting_first_update_ = false;
    if (!valid) {
      // The desk was asleep. Issuing a command now wakes it, but that first
      // command is consumed waking up and produces no movement, so timing
      // for stall/settle/coast starts only once a real height arrives.
      // Direction is not yet knowable (no height sample exists at all), so
      // the nudge direction is an arbitrary fixed choice; it never moves the
      // desk, it only wakes the link.
      state_ = MoveState::WAKING;
      command_ = MoveCommand::UP;
      return;
    }
    start_excursion_(height_cm, now_ms);
    return;
  }

  if (state_ == MoveState::WAKING) {
    if (!valid) {
      command_ = MoveCommand::UP;
      return;
    }
    start_excursion_(height_cm, now_ms);
    return;
  }

  if (!valid)
    return;   // lost the link mid-move; hold last command and wait

  if (state_ == MoveState::MOVING) {
    process_moving_sample_(height_cm, now_ms);
    return;
  }

  if (state_ == MoveState::SETTLING) {
    if (elapsed_at_least_(now_ms, settle_start_ms_, cfg_.settle_ms)) {
      float residual = height_cm - target_cm_;
      if (residual < 0)
        residual = -residual;
      if (cfg_.allow_correction && !correction_used_ && residual > deadband_at(height_cm)) {
        // Exactly one correction excursion is permitted per begin_move().
        // Direction is re-evaluated from where the desk actually is now, and
        // may differ from the initial direction (e.g. an overshoot), but
        // once locked here it is likewise never reversed mid-excursion.
        correction_used_ = true;
        start_excursion_(height_cm, now_ms);
      } else {
        state_ = MoveState::DONE;
        command_ = MoveCommand::NONE;
      }
    }
    return;
  }
}

}  // namespace ctb
