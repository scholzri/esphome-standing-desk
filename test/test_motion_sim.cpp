#include "../components/desk_link/motion_controller.h"
#include "desk_plant.h"
#include "harness.h"
#include <cmath>
#include <vector>

namespace {

struct Outcome {
  float final_height;
  float error;
  int reversals;
  int commanded_excursions;
  ctb::MoveState state;
};

Outcome run(float start, float target) {
  ctb::MotionConfig cfg;
  ctb::MotionController m;
  m.configure(cfg);
  DeskPlant plant(start);
  m.begin_move(target, 0);

  ctb::MoveCommand excursion_dir = ctb::MoveCommand::NONE;
  int reversals = 0, excursions = 0;

  for (uint32_t t = 0; t <= 60000; t += 40) {   // 40 ms, the real frame period
    m.update(plant.reported_height(), true, t);
    const ctb::MoveCommand c = m.command();
    if (c == ctb::MoveCommand::NONE) {
      excursion_dir = ctb::MoveCommand::NONE;      // excursion ended
    } else {
      if (excursion_dir == ctb::MoveCommand::NONE) {
        excursion_dir = c;                          // a new excursion began
        excursions++;
      } else if (c != excursion_dir) {
        reversals++;                                // direction flipped mid-flight
      }
    }
    plant.command(c);
    plant.tick(40);
    if (m.state() == ctb::MoveState::DONE || m.state() == ctb::MoveState::FAULT)
      break;
  }
  const float fh = plant.true_height();
  return {fh, std::fabs(fh - target), reversals, excursions, m.state()};
}

float bound_for(float target) { return target >= 100.0f ? 1.0f : 0.3f; }

}  // namespace

TEST(sim_meets_accuracy_across_the_range_from_below) {
  const float targets[] = {65.0f, 72.5f, 80.0f, 88.3f, 95.0f, 105.0f, 115.0f};
  for (float tgt : targets) {
    Outcome o = run(62.0f, tgt);
    CHECK(o.state == ctb::MoveState::DONE);
    CHECK(o.error <= bound_for(tgt));
    CHECK_EQ(o.reversals, 0);
  }
}

TEST(sim_meets_accuracy_across_the_range_from_above) {
  const float targets[] = {65.0f, 72.5f, 80.0f, 88.3f, 95.0f, 105.0f, 115.0f};
  for (float tgt : targets) {
    Outcome o = run(124.0f, tgt);
    CHECK(o.state == ctb::MoveState::DONE);
    CHECK(o.error <= bound_for(tgt));
    CHECK_EQ(o.reversals, 0);
  }
}

TEST(sim_never_needs_a_correction_when_calibrated) {
  // With the measured constants matching the plant, feed-forward alone should
  // land inside the deadband, so exactly one commanded excursion per move.
  const float targets[] = {70.0f, 85.0f, 95.0f};
  for (float tgt : targets) {
    CHECK_EQ(run(62.0f, tgt).commanded_excursions, 1);
    CHECK_EQ(run(99.0f, tgt).commanded_excursions, 1);
  }
}

TEST(sim_short_moves_still_land) {
  // A 2 cm move never reaches top speed, so the coast rule must scale with the
  // velocity actually achieved rather than assuming cruise.
  Outcome up = run(80.0f, 82.0f);
  CHECK(up.state == ctb::MoveState::DONE);
  CHECK(up.error <= 0.3f);
  Outcome down = run(82.0f, 80.0f);
  CHECK(down.state == ctb::MoveState::DONE);
  CHECK(down.error <= 0.3f);
}

TEST(sim_tolerates_coast_constants_being_off_by_a_third) {
  // Load on the desk changes the real coast. Accuracy may degrade but the
  // desk must never reverse or fault. Scaling coast(v) = latency*v +
  // v^2/(2*decel) uniformly by k at every velocity means latency *= k and
  // decel /= k (decel sits in the denominator of the term it controls).
  const float k = 1.33f;
  ctb::MotionConfig cfg;
  cfg.latency_up_s *= k;
  cfg.latency_down_s *= k;
  cfg.decel_up_cm_s2 /= k;
  cfg.decel_down_cm_s2 /= k;
  ctb::MotionController m; m.configure(cfg);
  DeskPlant plant(70.0f);
  m.begin_move(95.0f, 0);
  ctb::MoveCommand excursion_dir = ctb::MoveCommand::NONE;
  int reversals = 0;
  for (uint32_t t = 0; t <= 60000; t += 40) {
    m.update(plant.reported_height(), true, t);
    const ctb::MoveCommand c = m.command();
    if (c == ctb::MoveCommand::NONE) {
      excursion_dir = ctb::MoveCommand::NONE;
    } else {
      if (excursion_dir == ctb::MoveCommand::NONE)
        excursion_dir = c;
      else if (c != excursion_dir)
        reversals++;
    }
    plant.command(c); plant.tick(40);
    if (m.state() == ctb::MoveState::DONE || m.state() == ctb::MoveState::FAULT) break;
  }
  CHECK(m.state() == ctb::MoveState::DONE);
  CHECK_EQ(reversals, 0);
}
