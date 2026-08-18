#include "desk_cover.h"
#include "esphome/core/log.h"

namespace esphome {
namespace desk_link {

static const char *const TAG = "desk_cover";

void DeskCover::setup() {
  // Optimistic-free: position is always derived from the parent's live
  // height in loop() below, so there is nothing to restore from flash here.
  // current_operation defaults to COVER_OPERATION_IDLE (Cover's own
  // in-class initializer), which is correct until the first loop() sees a
  // moving controller state.
}

void DeskCover::loop() {
  if (!this->parent_->has_height())
    return;

  const float min = this->parent_->min_height_cm();
  const float max = this->parent_->max_height_cm();
  float position = cover::COVER_CLOSED;
  if (max > min) {
    position = (this->parent_->last_height_cm() - min) / (max - min);
    if (position < 0.0f)
      position = 0.0f;
    else if (position > 1.0f)
      position = 1.0f;
  }

  const auto state = this->parent_->controller().state();
  const bool moving = (state == ctb::MoveState::MOVING || state == ctb::MoveState::SETTLING ||
                       state == ctb::MoveState::WAKING);
  cover::CoverOperation op = cover::COVER_OPERATION_IDLE;
  if (moving) {
    // Direction is inferred from which way position needs to move, not from
    // MotionController's own direction_ (private, and meaningless anyway for
    // WAKING or for a box-side preset recall, which never touches
    // controller_ at all): current position vs. the controller's own
    // target_cm() is available and correct for the feed-forward case, and
    // during WAKING there is no target movement to report yet beyond "it's
    // busy", so OPENING is used as the least-wrong default the same way
    // MotionController's own WAKING nudge defaults to UP.
    const float target = this->parent_->controller().target_cm();
    op = (target < this->parent_->last_height_cm()) ? cover::COVER_OPERATION_CLOSING
                                                     : cover::COVER_OPERATION_OPENING;
  }

  // publish_state() is comparatively expensive (it touches RTC storage by
  // default) and loop() runs every pass; only publish when something a
  // subscriber would see has actually changed. save=false: position during
  // a move is not something worth surviving a reboot for, unlike a real
  // cover's rare open/close transitions -- the boot-time position is always
  // re-derived from the next live height report anyway.
  if (position != this->position || op != this->current_operation) {
    this->position = position;
    this->current_operation = op;
    this->publish_state(false);
  }
}

cover::CoverTraits DeskCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(false);
  // is_assumed_state left false (the default): position is always read back
  // from a live height report, never merely assumed from the last command.
  return traits;
}

void DeskCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    ESP_LOGI(TAG, "stop requested");
    this->parent_->controller().abort(millis());
    return;
  }
  if (call.get_position().has_value()) {
    const float min = this->parent_->min_height_cm();
    const float max = this->parent_->max_height_cm();
    const float target = min + *call.get_position() * (max - min);
    ESP_LOGI(TAG, "position %.2f requested -> target %.1f cm", *call.get_position(), target);
    this->parent_->move_to(target);
  }
}

}  // namespace desk_link
}  // namespace esphome
