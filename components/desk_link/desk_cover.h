#pragma once
#include "desk_link.h"
#include "esphome/components/cover/cover.h"

namespace esphome {
namespace desk_link {

// Presents the desk as an ESPHome cover::Cover. Position 0.0/1.0 map to the
// parent DeskLink's configured min_height_cm_/max_height_cm_; OPEN (1.0) is
// therefore the desk's maximum (standing) height and CLOSED (0.0) is its
// minimum (sitting) height. There is no real "closed" position (the desk
// never reaches 0 cm), which is exactly what a position-only cover with
// set_supports_toggle(false) already models correctly: HA's open/close
// affordances just become "go to max/go to min" rather than implying a
// literal fully-retracted state.
class DeskCover : public cover::Cover, public Component {
 public:
  void set_desk_link(DeskLink *parent) { this->parent_ = parent; }

  void setup() override;
  void loop() override;
  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;

  DeskLink *parent_{nullptr};
};

}  // namespace desk_link
}  // namespace esphome
