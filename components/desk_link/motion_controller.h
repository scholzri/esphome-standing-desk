#pragma once
#include <stdint.h>

namespace ctb {

enum class MoveCommand : uint8_t { NONE, UP, DOWN };
enum class MoveState : uint8_t { IDLE, WAKING, MOVING, SETTLING, DONE, FAULT };
enum class FaultReason : uint8_t { NONE, STALL, TIMEOUT, NO_HEIGHT, OUT_OF_RANGE, ABORTED };

struct MotionConfig {
  float min_height_cm{61.0f};
  float max_height_cm{126.0f};
  // Coast is t_latency * |v| + v^2 / (2 * decel): the distance covered during
  // the fixed command latency, plus the braking distance, which grows with the
  // square of speed. A purely proportional model cannot fit both short and
  // long moves, because a short move never reaches cruise speed.
  //
  // Seeded from hardware measurements: command-to-motion latency is 68 ms, and
  // coast was measured at 1.9 cm from 3.5 cm/s upward and 0.4 cm from
  // 3.9 cm/s downward. The box brakes roughly 15x harder descending, which is
  // why the decel figures differ so much.
  //
  // decel_up_cm_s2 is left at the value the 1.9 cm/3.5 cm/s measurement
  // implies. decel_down_cm_s2 is tuned to 19.0125 (down from an initial
  // 56.3, which over-read the 0.4 cm/3.9 cm/s measurement as already
  // including the latency contribution -- it does not, matching the plant's
  // own DECEL_DOWN_S derivation, v^2/(2*0.4) = 19.0125): see task-3 report,
  // "Fix: physical coast model". An exhaustive grid sweep (~975,000
  // (decel_up, decel_down) pairs, plus a further ~1.6M-point joint sweep
  // including both latencies) found a floor of exactly one failing
  // assertion, in sim_meets_accuracy_across_the_range_from_below at
  // tgt=115.0: the stop rule reads a spurious ~5 cm/s velocity from
  // aliasing between the plant's true 3.5 cm/s climb and the coarser 1 cm
  // display quantisation at and above 100 cm, and the SETTLING correction
  // that would otherwise fix it is skipped because the residual lands
  // exactly on the deadband_coarse_cm boundary (a strict "residual >
  // deadband" check against a quantised 1.0 cm residual). Neither is
  // reachable by retuning these four constants alone.
  float latency_up_s{0.068f};
  float latency_down_s{0.068f};
  float decel_up_cm_s2{3.69f};
  float decel_down_cm_s2{19.0125f};
  float deadband_cm{0.3f};          // used below 100 cm
  float deadband_coarse_cm{1.0f};   // at or above 100 cm: one quantisation step
  uint32_t wake_ms{250};
  uint32_t settle_ms{800};
  uint32_t stall_ms{1500};
  float stall_min_delta_cm{0.2f};
  uint32_t timeout_ms{40000};
  bool allow_correction{true};
};

// Drives a standing desk to a target height in one smooth travel.
//
// Pure logic: no ESPHome includes, no wall clock. Every call is given the
// current time explicitly as `now_ms`, matching the device's free-running
// millis() counter, so the same code runs on the desk and on the host under
// test. All elapsed-time arithmetic uses unsigned subtraction so it keeps
// working across a millis() rollover.
//
// State machine:
//
//   IDLE --begin_move(in range)--> [awaiting first update]
//     first update, valid=false --> WAKING --first valid update--> MOVING
//     first update, valid=true  ---------------------------------> MOVING
//   MOVING --stop rule fires--> SETTLING
//   SETTLING --settled within deadband, or correction already used--> DONE
//   SETTLING --residual outside deadband, correction allowed and unused-->
//     MOVING (single correction excursion; direction re-evaluated)
//   any active state --stall / timeout--> FAULT
//   any state --abort()--> FAULT (ABORTED)
//   begin_move(out of range) --> FAULT (OUT_OF_RANGE), no state entered
//
// Direction is locked once per excursion (the initial move, and again if a
// single correction is taken) and never reversed mid-excursion: the stop
// rule only ever silences the command, it does not flip it. That, plus the
// one-correction cap, is what keeps the desk from hunting.
class MotionController {
 public:
  void configure(const MotionConfig &cfg);
  void begin_move(float target_cm, uint32_t now_ms);
  void abort(uint32_t now_ms);
  // `valid` is false when the desk is asleep or no height frame has arrived.
  void update(float height_cm, bool valid, uint32_t now_ms);

  MoveCommand command() const { return command_; }
  MoveState state() const { return state_; }
  FaultReason fault() const { return fault_; }
  float target_cm() const { return target_cm_; }
  float velocity_cm_s() const { return velocity_cm_s_; }
  // The deadband that applies at a given height, accounting for the coarser
  // reporting resolution at and above 100 cm.
  float deadband_at(float height_cm) const;

 private:
  // Ring buffer capacity for the velocity window. Not the window's duration
  // (see VEL_WINDOW_MS in motion_controller.cpp) -- just how many samples of
  // history can be held at once, generous enough to cover VEL_WINDOW_MS at
  // any sample rate the desk is expected to run at.
  static const uint8_t VEL_CAPACITY = 32;

  void enter_fault_(FaultReason reason);
  void start_excursion_(float from_height_cm, uint32_t now_ms);
  // Records a sample, updates stall tracking, and applies the feed-forward
  // stop rule. Called for every sample taken while MOVING, including the
  // first one of an excursion (right after start_excursion_()), so the
  // zero-velocity guard on the stop rule is exercised on that first sample
  // too, not just on later ones.
  void process_moving_sample_(float height_cm, uint32_t now_ms);
  void record_sample_(float height_cm, uint32_t now_ms);
  void reset_velocity_window_();
  bool elapsed_at_least_(uint32_t now_ms, uint32_t since_ms, uint32_t duration_ms) const;
  float remaining_cm_(float height_cm) const;

  MotionConfig cfg_{};

  MoveCommand command_{MoveCommand::NONE};
  MoveState state_{MoveState::IDLE};
  FaultReason fault_{FaultReason::NONE};
  float target_cm_{0.0f};
  float velocity_cm_s_{0.0f};

  // True from begin_move() until the first update() resolves WAKING vs MOVING.
  bool awaiting_first_update_{false};

  MoveCommand direction_{MoveCommand::NONE};   // locked command for the current excursion
  uint32_t move_start_ms_{0};                  // begin_move() time; drives timeout_ms
  uint32_t settle_start_ms_{0};                // time SETTLING was entered
  bool correction_used_{false};                // at most one per begin_move()

  // Stall tracking: height and time of the last point progress was seen.
  float stall_anchor_cm_{0.0f};
  uint32_t stall_anchor_ms_{0};
  bool stall_anchor_set_{false};

  // Sliding window of recent (time, height) samples for velocity estimation.
  float vel_h_[VEL_CAPACITY]{};
  uint32_t vel_t_[VEL_CAPACITY]{};
  uint8_t vel_count_{0};
  uint8_t vel_head_{0};   // next slot to write
};

}  // namespace ctb
