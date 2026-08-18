#include "../components/desk_link/motion_controller.h"
#include "harness.h"

namespace {

ctb::MotionConfig cfg() {
  ctb::MotionConfig c;   // defaults are the measured constants
  return c;
}

// Feeds a height at a timestamp and returns the resulting command.
ctb::MoveCommand step(ctb::MotionController &m, float h, uint32_t t) {
  m.update(h, true, t);
  return m.command();
}

}  // namespace

TEST(rejects_target_above_max) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(200.0f, 0);
  CHECK(m.state() == ctb::MoveState::FAULT);
  CHECK(m.fault() == ctb::FaultReason::OUT_OF_RANGE);
  CHECK(m.command() == ctb::MoveCommand::NONE);
}

TEST(rejects_target_below_min) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(10.0f, 0);
  CHECK(m.state() == ctb::MoveState::FAULT);
  CHECK(m.fault() == ctb::FaultReason::OUT_OF_RANGE);
}

TEST(wakes_before_moving_when_height_invalid) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  m.update(0.0f, false, 10);           // asleep: no valid height
  CHECK(m.state() == ctb::MoveState::WAKING);
  CHECK(m.command() != ctb::MoveCommand::NONE);   // a nudge is issued
  m.update(80.0f, true, 400);                     // desk woke, height arrives
  CHECK(m.state() == ctb::MoveState::MOVING);
}

TEST(commands_up_toward_a_higher_target) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  CHECK_EQ((int) step(m, 80.0f, 10), (int) ctb::MoveCommand::UP);
}

TEST(commands_down_toward_a_lower_target) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(70.0f, 0);
  CHECK_EQ((int) step(m, 80.0f, 10), (int) ctb::MoveCommand::DOWN);
}

TEST(does_not_stop_on_a_zero_velocity_estimate_at_the_start) {
  // With one sample velocity is zero, so coast distance is zero and a naive
  // rule would stop immediately even though the desk has not moved.
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  CHECK_EQ((int) step(m, 80.0f, 10), (int) ctb::MoveCommand::UP);
  CHECK(m.state() == ctb::MoveState::MOVING);
}

TEST(stops_short_by_the_upward_coast_distance) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  // Climb at 3.5 cm/s: 0.35 cm every 100 ms.
  float h = 80.0f;
  uint32_t t = 0;
  ctb::MoveCommand last = ctb::MoveCommand::NONE;
  while (t < 20000 && m.state() == ctb::MoveState::MOVING) {
    last = step(m, h, t);
    if (last == ctb::MoveCommand::NONE) break;
    h += 0.35f; t += 100;
  }
  // coast_up_s 0.543 * 3.5 cm/s = about 1.9 cm of lead.
  CHECK(h >= 87.5f);
  CHECK(h <= 88.6f);
  CHECK(m.state() == ctb::MoveState::SETTLING);
}

TEST(stops_much_later_going_down_because_coast_is_smaller) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(80.0f, 0);
  float h = 90.0f;
  uint32_t t = 0;
  while (t < 20000 && m.state() == ctb::MoveState::MOVING) {
    if (step(m, h, t) == ctb::MoveCommand::NONE) break;
    h -= 0.39f; t += 100;   // 3.9 cm/s downward
  }
  // coast_down_s 0.103 * 3.9 = about 0.4 cm of lead, so it runs far closer in.
  CHECK(h <= 80.7f);
  CHECK(h >= 79.5f);
}

TEST(never_reverses_direction) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  step(m, 80.0f, 0);
  step(m, 85.0f, 1000);
  // Overshoot past the target.
  // NOTE: the brief's source uses a comma operator here
  // (`CHECK(...), step(...)`), which does not compile: CHECK's do/while(0)
  // expands to a compound statement, and the comma operator cannot sequence
  // a statement. Rewritten as two statements preserving the same assertion,
  // per the task instructions for this exact case.
  for (uint32_t t = 2000; t < 4000; t += 100) {
    CHECK(m.command() != ctb::MoveCommand::DOWN);
    step(m, 95.0f, t);
  }
}

TEST(deadband_is_coarser_above_one_metre) {
  ctb::MotionController m; m.configure(cfg());
  CHECK(m.deadband_at(90.0f) < 0.5f);
  CHECK(m.deadband_at(100.0f) >= 1.0f);
  CHECK(m.deadband_at(115.0f) >= 1.0f);
}

TEST(faults_on_stall) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  step(m, 80.0f, 0);
  for (uint32_t t = 100; t <= 3000; t += 100)
    step(m, 80.0f, t);          // commanded to move, but nothing happens
  CHECK(m.state() == ctb::MoveState::FAULT);
  CHECK(m.fault() == ctb::FaultReason::STALL);
  CHECK(m.command() == ctb::MoveCommand::NONE);
}

TEST(faults_on_timeout) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(126.0f, 0);
  float h = 61.0f;
  for (uint32_t t = 0; t <= 45000; t += 100) {
    step(m, h, t);
    h += 0.01f;   // crawling, below the stall threshold per window but never arriving
    if (m.state() == ctb::MoveState::FAULT) break;
  }
  CHECK(m.state() == ctb::MoveState::FAULT);
}

TEST(abort_stops_immediately) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  step(m, 80.0f, 0);
  m.abort(500);
  CHECK(m.state() == ctb::MoveState::FAULT);
  CHECK(m.fault() == ctb::FaultReason::ABORTED);
  CHECK(m.command() == ctb::MoveCommand::NONE);
}

TEST(at_most_one_correction_per_move) {
  ctb::MotionController m; m.configure(cfg());
  m.begin_move(90.0f, 0);
  step(m, 80.0f, 0);
  // Jump straight past the target so a correction is warranted.
  step(m, 93.0f, 1000);
  uint32_t t = 1000;
  int excursions = 0;
  ctb::MoveCommand prev = ctb::MoveCommand::NONE;
  while (t < 30000 && m.state() != ctb::MoveState::DONE &&
         m.state() != ctb::MoveState::FAULT) {
    t += 100;
    ctb::MoveCommand c = step(m, 93.0f, t);
    if (c != ctb::MoveCommand::NONE && prev == ctb::MoveCommand::NONE)
      excursions++;
    prev = c;
  }
  CHECK(excursions <= 1);
}
