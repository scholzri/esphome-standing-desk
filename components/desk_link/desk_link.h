#pragma once
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "frame_codec.h"
#include "height_codec.h"
#include "motion_controller.h"

namespace esphome {
namespace desk_link {

// Fires with the real command byte whenever a genuine keypad frame (not the
// 0x40 idle shape, not a hold-override rewrite) passes through
// handle_keypad_byte_(). Registered from __init__.py the same way
// matrix_keypad's MatrixKeyTrigger is: a Trigger<uint8_t> subclass held in a
// vector on the parent component and fired with .trigger(cmd_byte).
//
// This is the current ESPHome idiom for a component-level `on_..._byte`
// automation. An earlier design note called for a hand-rolled
// CallbackManager<void(uint8_t)> member instead; that class does exist in
// this pinned ESPHome (2026.6.5, confirmed in esphome/core/helpers.h), but
// Trigger<uint8_t> is the idiom actually used for this shape of problem --
// it is exactly what ships in esphome/components/matrix_keypad for the same
// case (an on_key trigger carrying one uint8_t), and matching an existing
// component's pattern beats introducing a second way to do the same thing.
class DeskKeypadPressTrigger : public Trigger<uint8_t> {};

class DeskLink : public Component {
 public:
  void set_keypad_uart(uart::UARTComponent *u) { this->keypad_uart_ = u; }
  void set_controlbox_uart(uart::UARTComponent *u) { this->controlbox_uart_ = u; }
  void set_capture_port(uint16_t p) { this->capture_port_ = p; }
  void set_stall_guard_ms(uint32_t ms) { this->stall_guard_ms_ = ms; }
  void set_min_height_cm(float cm) { this->min_height_cm_ = cm; }
  void set_max_height_cm(float cm) { this->max_height_cm_ = cm; }
  // Readable alongside the existing write-only setters above: desk_cover
  // needs the configured range to map a HA 0..1 position to/from a target
  // height (see DeskCover::control() and its position feedback in loop()),
  // and nothing on DeskLink exposed it before this was written.
  float min_height_cm() const { return this->min_height_cm_; }
  float max_height_cm() const { return this->max_height_cm_; }

  // Optional: when set, handle_box_byte_() publishes decoded heights
  // directly to this sensor, at line rate (checked on every box height
  // frame, i.e. the box's own ~40 ms cadence, and published whenever the
  // value changes) rather than the 500 ms poll the old standing_desk_height
  // component used. desk_link now owns both
  // UARTs exclusively (see the class comment history in desk_link.cpp), so
  // a separate polling sensor reading the same controlbox UART id is no
  // longer possible; this replaces that role directly instead. Optional
  // (nullptr-checked before use) so desk_link stays usable, e.g. in
  // sniffer.yaml, without a sensor entity configured at all.
  void set_height_sensor(sensor::Sensor *s) { this->height_sensor_ = s; }

  // Live-tunable MotionConfig fields. Each mutates the DeskLink-held
  // motion_cfg_ (seeded from min/max height in setup(), the same values
  // MotionConfig itself defaults to) and re-applies it via
  // controller_.configure(), which already replaces the whole config
  // wholesale (`cfg_ = cfg;` in motion_controller.cpp) -- that existing
  // behaviour is the entire live-reconfigure mechanism; no new
  // MotionController API is needed. Kept as four setters rather than one
  // taking a struct so each can be wired to its own `number:` entity with
  // its own `on_value` lambda in base.yaml, matching how min/max height are
  // already exposed individually.
  void set_latency_up_s(float s) {
    this->motion_cfg_.latency_up_s = s;
    this->controller_.configure(this->motion_cfg_);
  }
  void set_latency_down_s(float s) {
    this->motion_cfg_.latency_down_s = s;
    this->controller_.configure(this->motion_cfg_);
  }
  void set_decel_up_cm_s2(float v) {
    this->motion_cfg_.decel_up_cm_s2 = v;
    this->controller_.configure(this->motion_cfg_);
  }
  void set_decel_down_cm_s2(float v) {
    this->motion_cfg_.decel_down_cm_s2 = v;
    this->controller_.configure(this->motion_cfg_);
  }

  // Diagnostic counters. All were already tracked internally (and reported
  // over the capture socket's STAT line); these just make them readable from
  // codegen'd C++, e.g. a sensor's lambda.
  uint32_t drops() const { return this->drops_; }
  uint32_t stall_flushes() const { return this->stall_flushes_; }
  uint32_t full_buffer_flushes() const { return this->full_buffer_flushes_; }
  uint32_t keypad_crc_errors() const { return this->keypad_codec_.crc_errors(); }
  uint32_t keypad_resyncs() const { return this->keypad_codec_.resyncs(); }
  uint32_t box_crc_errors() const { return this->box_codec_.crc_errors(); }
  uint32_t box_resyncs() const { return this->box_codec_.resyncs(); }

  // Last height decoded from the box stream, and whether one has ever been
  // seen. handle_box_byte_() already computes this on every box height frame
  // (to feed controller_.update()) but previously discarded it; it is now
  // cached here as the one place DeskLink tracks "the current height" for
  // anything that isn't the motion controller itself -- the cover's position
  // feedback and the line-rate height sensor both read it from here rather
  // than each keeping their own copy.
  float last_height_cm() const { return this->last_height_cm_; }
  bool has_height() const { return this->has_height_; }

  // Human-readable form of controller().fault(), for a diagnostic text
  // sensor. Returned as a static string literal; never allocates.
  const char *fault_reason_string() const;

  void register_keypad_press_trigger(DeskKeypadPressTrigger *trig) {
    this->keypad_press_triggers_.push_back(trig);
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Sends `data` to the control box `count` times, `interval_ms` apart, for
  // genuine one-shot frames the keypad itself would only ever send once, such
  // as preset recall. This no longer suppresses the keypad stream: the byte
  // interleaving is left to hardware arbitration on the shared UART, exactly
  // as two independent writers would behave. Anything longer than
  // MAX_FRAME_LEN is rejected, which makes an over-long injection impossible
  // by construction rather than by convention.
  void inject(const uint8_t *data, uint8_t len, uint16_t count, uint16_t interval_ms);

  // Rewrites the command byte of every subsequent idle keypad frame
  // (AA FF xx 40 ..) to `cmd` for `duration_ms`, as if a key were
  // physically held. `param` rewrites the param byte alongside it, since
  // some button presses (preset recall/store) are only distinguishable by
  // their param byte, not their command byte alone. duration_ms == 0 cancels
  // immediately (RELEASE). duration_ms is capped internally so a dropped
  // capture connection cannot leave the desk driving indefinitely.
  void set_hold(uint8_t cmd, uint8_t param, uint32_t duration_ms);
  void recall_preset(uint8_t index);   // index 0-3, zero-based on the wire
  // Stores the desk's CURRENT height into preset `index` (0-3) on the
  // control box itself -- the protocol has no way to store an arbitrary
  // height, only "whatever height it's at right now". Mirrors the
  // STOREPRESET capture-socket verb (0x44 = base 0x40 | store bit 0x04, see
  // handle_command_()), exposed here as a public method so YAML button
  // entities can call it directly instead of only being reachable from the
  // socket.
  void store_preset(uint8_t index);

  // Records the measured height of stored preset `index` (0-3), so move_to()
  // can recognise when a requested target matches it. 0 means unknown/unset.
  void set_preset_height(uint8_t index, float height_cm);
  // Moves the desk to `target_cm`. Delegates to the box's own preset
  // positioning when the target is within PRESET_MATCH_CM of a known preset
  // height (it stops exactly on target using its own deceleration ramp,
  // which feed-forward cannot match); otherwise drives the feed-forward
  // controller_. If a feed-forward move is already under way (MOVING,
  // SETTLING or WAKING), that excursion is aborted first so a new request
  // always supersedes an old one cleanly rather than racing it -- see the
  // definition for why this matters now that both the cover and the target
  // height number call in here.
  void move_to(float target_cm);
  ctb::MotionController &controller() { return this->controller_; }

 protected:
  void handle_keypad_byte_(uint8_t b);
  void handle_box_byte_(uint8_t b);
  void emit_frame_(ctb::Direction dir, const ctb::Frame &f);
  void flush_hold_(const char *reason);
  void service_hold_expiry_();
  void service_injection_();
  void service_capture_();
  void handle_command_(char *line);
  void capture_write_(const char *s, size_t len);

  uart::UARTComponent *keypad_uart_{nullptr};
  uart::UARTComponent *controlbox_uart_{nullptr};

  ctb::FrameCodec keypad_codec_{ctb::Direction::KEYPAD_TO_BOX};
  ctb::FrameCodec box_codec_{ctb::Direction::BOX_TO_KEYPAD};

  // Feed-forward motion controller, configured from min_height_cm_/
  // max_height_cm_ in setup() and driven from every decoded box height frame
  // in handle_box_byte_(). move_to() delegates to recall_preset() instead of
  // this controller when the target matches a stored preset height.
  ctb::MotionController controller_;
  float preset_heights_[4]{0, 0, 0, 0};   // 0 means unknown
  static constexpr float PRESET_MATCH_CM = 0.6f;

  // DeskLink's own copy of the config last pushed to controller_, so a
  // single-field change (from a `number:` entity's on_value, e.g.
  // set_decel_up_cm_s2()) can be folded in and reapplied without callers
  // having to know or reconstruct the other three fields. Seeded in setup()
  // from min_height_cm_/max_height_cm_ exactly as before; the four
  // latency/decel fields start at MotionConfig's own defaults until a
  // set_*() call overrides one.
  ctb::MotionConfig motion_cfg_{};

  // Last height decoded in handle_box_byte_(), cached for callers that need
  // "the current height" outside the motion controller (desk_cover's
  // position feedback, the line-rate height sensor). has_height_ is false
  // until the first valid box height frame arrives (mirrors decode_height_
  // frame()'s own negative-means-invalid convention, but as a bool rather
  // than a sentinel float so 0 cm, if ever reached, is not mistaken for
  // "unknown").
  float last_height_cm_{0.0f};
  bool has_height_{false};
  sensor::Sensor *height_sensor_{nullptr};   // optional; see set_height_sensor()

  // Registered on_keypad_press automations (see DeskKeypadPressTrigger).
  // Fired from handle_keypad_byte_() at the same point that already detects
  // a genuine (non-idle, non-override) keypress for the hands-always-win
  // check, so there is exactly one detection point for "was this a real
  // press", not two.
  std::vector<DeskKeypadPressTrigger *> keypad_press_triggers_;

  uint16_t capture_port_{6789};
  int listen_fd_{-1};
  int client_fd_{-1};
  char cmd_buf_[192];
  uint8_t cmd_len_{0};

  // How long held keypad bytes may sit unforwarded before the stall guard
  // (see loop()) flushes them unmodified. Defaults to 30 ms in __init__.py;
  // see the comment there for why 30 ms is the chosen value.
  uint32_t stall_guard_ms_{30};

  // Range passed to controller_.configure() in setup(). Defaults match
  // MotionConfig's own (61.0 / 126.0); configurable from __init__.py as
  // min_height / max_height.
  float min_height_cm_{61.0f};
  float max_height_cm_{126.0f};

  uint8_t inject_buf_[ctb::MAX_FRAME_LEN];
  uint8_t inject_len_{0};
  uint16_t inject_remaining_{0};
  uint16_t inject_interval_ms_{38};
  uint32_t inject_next_ms_{0};

  // Keypad-to-box store-and-forward. Every keypad byte -- there is no
  // byte-immediate path any more -- is appended to hold_buf_ and fed to
  // keypad_codec_ in lockstep, so hold_buf_[0 .. hold_len_) always holds
  // exactly the bytes of the frame keypad_codec_ is currently assembling
  // (or has just completed), whether or not any of it has already been
  // written out.
  //
  // hold_len_ alone cannot tell you whether the codec is mid-frame: the
  // stall guard (and the full-buffer guard) can write held bytes out to the
  // control box while the codec is still genuinely mid-frame, and neither
  // may reset hold_len_ to 0 at that point without losing track of the
  // frame's already-buffered bytes. So a second counter, hold_sent_, tracks
  // how many bytes of the *current* frame have already been written to the
  // control box. hold_sent_ <= hold_len_ always. On frame close:
  //   - hold_sent_ == 0 means nothing of this frame went out yet, so the
  //     whole frame is still eligible for rewrite.
  //   - hold_sent_ > 0 means a prefix already left the building verbatim;
  //     only the unsent tail hold_buf_[hold_sent_ .. hold_len_) is forwarded,
  //     never rewritten, since bytes already on the wire can't be recalled.
  // Both counters reset to 0 only once a frame is fully accounted for
  // (closed-and-forwarded) or the full-buffer guard forces a hard reset.
  uint8_t hold_buf_[ctb::MAX_FRAME_LEN];
  uint8_t hold_len_{0};
  uint8_t hold_sent_{0};
  uint32_t hold_start_ms_{0};

  // Command-byte override ("hold"), applied only to idle (cmd 0x40) frames.
  bool override_active_{false};
  uint8_t override_cmd_{0};
  uint8_t override_param_{0x00};
  uint32_t override_deadline_ms_{0};

  // True only while the currently active override was set by this
  // component's own controller-command dispatch (handle_box_byte_'s UP/DOWN
  // cases), never by any other caller (a raw HOLD/RELEASE socket command,
  // recall_preset, store_preset). The controller reports NONE as its default
  // resting state -- true whenever no move is actively driving, which is
  // almost always -- so releasing on NONE unconditionally would kill any hold
  // this dispatch did not itself create. This flag is what lets the release
  // logic tell those two cases apart.
  bool override_owned_by_controller_{false};

  uint32_t seq_{0};
  uint32_t drops_{0};

  // Counts of flush_hold_() calls by cause, reported in STAT so a workstation
  // script can confirm from outside how often the stall guard is firing --
  // it should be rare at the default 30 ms, and every firing means some
  // frame's tail was forwarded unrewritten (see flush_hold_ and the
  // stall-guard check in loop()).
  uint32_t stall_flushes_{0};
  uint32_t full_buffer_flushes_{0};
};

}  // namespace desk_link
}  // namespace esphome
