#include "desk_link.h"
#include "ctb_crc.h"
#include "esphome/core/log.h"

#include <errno.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace esphome {
namespace desk_link {

static const char *const TAG = "desk_link";

void DeskLink::setup() {
  // motion_cfg_ (not a throwaway local) so later live-tunable setters
  // (set_latency_up_s() etc.) can mutate one field and reapply the whole
  // struct without needing to already know the other three.
  this->motion_cfg_.min_height_cm = this->min_height_cm_;
  this->motion_cfg_.max_height_cm = this->max_height_cm_;
  this->controller_.configure(this->motion_cfg_);

  this->listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (this->listen_fd_ < 0) {
    ESP_LOGE(TAG, "socket() failed: errno %d", errno);
    return;
  }
  int on = 1;
  ::setsockopt(this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->capture_port_);

  if (::bind(this->listen_fd_, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "bind(%u) failed: errno %d", this->capture_port_, errno);
    ::close(this->listen_fd_);
    this->listen_fd_ = -1;
    return;
  }
  ::listen(this->listen_fd_, 1);
  ::fcntl(this->listen_fd_, F_SETFL, O_NONBLOCK);
  ESP_LOGI(TAG, "capture socket listening on port %u", this->capture_port_);
}

void DeskLink::loop() {
  // Drained in chunks rather than a byte at a time: every available(),
  // read_byte() and write_byte() is its own ESP-IDF driver call with its own
  // locking, so per-byte I/O cost three driver round trips per byte. Bytes
  // are still handled one by one, in arrival order, exactly as before.
  uint8_t buf[64];
  size_t n;
  while ((n = this->keypad_uart_->available()) > 0) {
    if (n > sizeof(buf))
      n = sizeof(buf);
    if (!this->keypad_uart_->read_array(buf, n))
      break;
    for (size_t i = 0; i < n; i++)
      this->handle_keypad_byte_(buf[i]);
  }
  while ((n = this->controlbox_uart_->available()) > 0) {
    if (n > sizeof(buf))
      n = sizeof(buf);
    if (!this->controlbox_uart_->read_array(buf, n))
      break;
    // The box stream is always forwarded unchanged, before any of it is
    // decoded, so the keypad display never waits on the decode path and
    // never goes dark.
    this->keypad_uart_->write_array(buf, n);
    for (size_t i = 0; i < n; i++)
      this->handle_box_byte_(buf[i]);
  }
  // Checked every pass, not only when a keypad byte just arrived: the stall
  // guard bounds wall-clock time held, and the keypad can simply stop
  // sending mid-frame (or forever). This is the one check in this file that
  // must never be skipped, because a stall here is exactly the condition
  // that faulted the control box: a gap in the 41 ms keypad stream.
  // Guarded on hold_sent_ < hold_len_ (there is unsent, held data right
  // now), not hold_len_ > 0: flush_hold_ no longer resets hold_len_ (it must
  // keep the frame's bytes available for handle_keypad_byte_'s completion
  // logic), so hold_len_ alone would stay truthy long after everything held
  // has already been flushed, retriggering this every single pass.
  //
  // stall_guard_ms_ defaults to 30 (see __init__.py for the full reasoning):
  // it must sit well above ESPHome's own loop-scale timing -- a 6-byte
  // keypad frame occupies only ~3.1 ms of wire time at 19200 baud, well
  // under the loop's own ~16 ms drain cadence, so completing a frame that
  // starts being held never legitimately takes longer than about one loop
  // period; a held frame still incomplete well past that is a genuine gap in
  // delivery, not routine timing -- while staying below the keypad's 41 ms
  // frame period, so the control box still receives frames within one
  // cadence period even after a flush. 10 ms cut it too close to that ~16 ms
  // loop-period scale for brief, real delivery gaps (a busy WiFi/API/OTA/
  // capture-socket cooperative loop schedule, not a hardware fault) to clear
  // without tripping the guard; 30 ms gives real margin while staying more
  // than an order of magnitude under the ~1 s blank that used to fault the
  // control box. The subtraction handles millis() rollover the same way the
  // deadline checks elsewhere in this file do.
  if (this->hold_sent_ < this->hold_len_ &&
      (uint32_t) (millis() - this->hold_start_ms_) >= this->stall_guard_ms_) {
    this->flush_hold_("stall guard");
    this->stall_flushes_++;
  }
  this->service_hold_expiry_();
  this->service_injection_();
  this->service_capture_();
}

void DeskLink::handle_box_byte_(uint8_t b) {
  ctb::Frame f;
  if (!this->box_codec_.put(b, &f))
    return;
  this->emit_frame_(ctb::Direction::BOX_TO_KEYPAD, f);

  // Negative means "not a height report": a button frame, a malformed frame,
  // or the sleep sentinel. The desk being asleep is exactly the invalid-height
  // case the controller's wake handling exists for.
  const float h = ctb::decode_height_frame(f);
  const bool valid = (h >= 0.0f);
  if (valid) {
    // Published at line rate (not on a poll timer), but only when the value
    // actually changed. The box repeats its height report every ~42 ms even
    // while the desk is idle, and Sensor::publish_state() does not dedupe,
    // so publishing every frame meant ~23 identical states per second, around
    // the clock, into the API, web_server and any on_value automations. The
    // captures in test/fixtures bear this out: idle.log carries 1394 height
    // frames and one distinct value; manual_up_press.log carries 1328 frames
    // and 40 changes. Comparing exactly is safe: decode_height_frame() builds
    // every value from the same integer digits, so equal frames give equal
    // floats. Checked before last_height_cm_ is overwritten below.
    const bool changed = !this->has_height_ || h != this->last_height_cm_;
    // Cached here, not just fed to controller_.update(), so anything that
    // needs "the current height" but isn't the motion controller itself
    // (desk_cover's position feedback) has one place to read it rather than
    // each keeping its own copy in step with this same decode.
    this->last_height_cm_ = h;
    this->has_height_ = true;
    if (changed && this->height_sensor_ != nullptr)
      this->height_sensor_->publish_state(h);
  }
  this->controller_.update(valid ? h : 0.0f, valid, millis());

  switch (this->controller_.command()) {
    case ctb::MoveCommand::UP:
      this->set_hold(0x50, 0x00, 200);
      this->override_owned_by_controller_ = true;
      break;
    case ctb::MoveCommand::DOWN:
      this->set_hold(0x60, 0x00, 200);
      this->override_owned_by_controller_ = true;
      break;
    case ctb::MoveCommand::NONE:
      // controller_.command() reports NONE as its default resting state --
      // true whenever no move is actively driving, which is almost always,
      // including idle, done, faulted, or simply never invoked. Releasing on
      // NONE unconditionally would kill ANY active override the instant this
      // ran, not just one this dispatch itself set: a raw HOLD sent over the
      // capture socket for testing, a preset recall, or the controller's own
      // hold during any transient moment it reported NONE. Gating on
      // override_owned_by_controller_ -- true only in the narrow window right
      // after the UP/DOWN cases above set it -- means this only ever
      // releases a hold this exact dispatch created. A preset recall
      // (recall_preset, cmd 0x48) is never set from inside this switch, so
      // the flag is never true for it either; the old override_cmd_ != 0x48
      // check this replaced is therefore no longer needed.
      if (this->override_active_ && this->override_owned_by_controller_) {
        this->set_hold(0x40, 0x00, 0);
        this->override_owned_by_controller_ = false;
      }
      break;
  }
}

void DeskLink::handle_keypad_byte_(uint8_t b) {
  // A byte can only be rewritten before it is sent, so keypad->box traffic
  // is held until a frame closes rather than written byte-immediate. There
  // is no separate byte-immediate path any more: every keypad byte, sync or
  // not, in-frame or not, is appended to hold_buf_ and fed to keypad_codec_.
  // Frame state (has the codec seen a complete frame yet, and how much of it
  // has already been written to the control box) is tracked explicitly by
  // hold_len_/hold_sent_, not inferred from whether hold_len_ happens to be
  // zero -- hold_len_ == 0 does NOT mean "not mid-frame": the stall guard
  // and the full-buffer guard can both flush held bytes to the wire while
  // keypad_codec_ is still genuinely assembling a frame.
  //
  // hold_buf_ is the same size as keypad_codec_'s own frame buffer
  // (MAX_FRAME_LEN), but the codec only checks its own bound on the byte
  // *after* this one arrives. Flushing here, before a full buffer is asked
  // to hold a 65th byte, keeps hold_buf_ from ever being written out of
  // bounds. This never triggers on real desk traffic -- the largest real
  // frame is 28 bytes, and the stall guard (default 30 ms, well under the
  // keypad's 41 ms frame period) would ordinarily flush long before 64 bytes
  // ever accumulate -- but nothing here may depend on "ordinarily".
  if (this->hold_len_ >= ctb::MAX_FRAME_LEN) {
    this->flush_hold_("hold buffer full");
    this->full_buffer_flushes_++;
    this->hold_len_ = 0;
    this->hold_sent_ = 0;
  }

  if (this->hold_len_ == 0)
    this->hold_start_ms_ = millis();
  this->hold_buf_[this->hold_len_++] = b;

  // keypad_codec_ has no public "am I mid-frame" query, but resyncs() is a
  // running count that increments in exactly one place: a byte arrives while
  // the codec is not in a frame and is not the sync byte, so the codec
  // discards it as noise without ever absorbing it into any frame (see
  // FrameCodec::put in frame_codec.cpp). Watching that counter's delta
  // across this one put() call is how a byte the codec just rejected as
  // pure inter-frame noise is told apart from a byte that is genuinely
  // accumulating toward a frame. That distinction matters here because
  // hold_buf_[0 .. hold_len_) must contain *only* bytes the codec considers
  // part of the frame currently in progress or just completed -- a noise
  // byte left sitting in hold_buf_ ahead of the next real frame would throw
  // off hold_len_/hold_sent_ bookkeeping for that frame (wrong rewrite
  // offset, wrong CRC base, wrong forwarded length), even though the byte
  // itself would still reach the box.
  const uint32_t resyncs_before = this->keypad_codec_.resyncs();
  ctb::Frame f;
  const bool closed = this->keypad_codec_.put(b, &f);

  if (!closed) {
    if (this->keypad_codec_.resyncs() != resyncs_before) {
      // This byte was never part of any frame; forward it immediately and
      // drop it back out of hold_buf_/hold_len_ so it can't be mistaken for
      // a leading byte of whatever frame comes next.
      this->controlbox_uart_->write_byte(b);
      this->hold_len_--;
      if (this->hold_len_ == 0)
        this->hold_sent_ = 0;
    }
    return;
  }

  // hold_sent_ == 0 means nothing of this now-complete frame has been
  // written yet, so the whole frame is still eligible for rewrite.
  // hold_sent_ > 0 means a prefix already left the building verbatim (a
  // stall-guard or full-buffer flush fired mid-frame); only the unsent tail
  // is forwarded, and it is forwarded unmodified, because bytes already on
  // the wire can never be recalled or retroactively changed.
  const bool whole_frame_pending = (this->hold_sent_ == 0);
  const bool is_idle_shaped = (f.len == 6 && f.bytes[1] == 0xFF);

  if (whole_frame_pending && this->override_active_ && is_idle_shaped &&
      f.bytes[3] == 0x40) {
    // Both bytes matter: the command byte is a button bitmask and the param
    // byte carries the preset index (zero-based). Recalling preset 3 is
    // AA FF 02 48, so rewriting the command alone can only ever reach
    // preset 1.
    this->hold_buf_[2] = this->override_param_;
    this->hold_buf_[3] = this->override_cmd_;
    ctb::crc_append(this->hold_buf_, 4);
    this->controlbox_uart_->write_array(this->hold_buf_, this->hold_len_);
  } else {
    this->controlbox_uart_->write_array(this->hold_buf_ + this->hold_sent_,
                                        this->hold_len_ - this->hold_sent_);
  }

  // Hands always win. Runs on every completed frame regardless of which
  // branch above ran, and regardless of whether this check is seeing the
  // frame in full or only a tail of it (f.bytes[3] is the codec's own
  // parsed copy of the command byte, never mutated by the rewrite above): a
  // real keypress always cancels any active automated hold immediately.
  //
  // This is also the one place a genuine keypad frame (not 0x40 idle, not a
  // hold-override rewrite -- f.bytes[3] is the codec's parsed copy,
  // unaffected by the rewrite above) is already distinguished from idle
  // filler, so on_keypad_press fires from here rather than adding a second,
  // possibly-diverging detection point.
  if (is_idle_shaped && f.bytes[3] != 0x40) {
    if (this->override_active_) {
      this->override_active_ = false;
      // A real keypress cancels the hold outright, regardless of who set it,
      // so any "owned by controller" status it carried is no longer
      // meaningful and must not survive to be inherited by whatever hold
      // gets set next.
      this->override_owned_by_controller_ = false;
      ESP_LOGI(TAG, "keypad cmd 0x%02X cancels active hold", f.bytes[3]);
    }
    for (auto *trig : this->keypad_press_triggers_)
      trig->trigger(f.bytes[3]);
  }

  // Surface the frame on the capture stream. `f` is the codec's own copy and
  // is never touched by the rewrite above, which mutates hold_buf_ instead, so
  // this reports what the keypad genuinely sent rather than what was forwarded
  // to the box. That is the direction protocol capture needs: what we injected
  // is already known, what the keypad said is the thing being discovered.
  // Without this the entire keypad->box direction is invisible to capture.
  this->emit_frame_(ctb::Direction::KEYPAD_TO_BOX, f);

  this->hold_len_ = 0;
  this->hold_sent_ = 0;
}

void DeskLink::flush_hold_(const char *reason) {
  if (this->hold_sent_ >= this->hold_len_)
    return;
  const uint8_t n = this->hold_len_ - this->hold_sent_;
  ESP_LOGW(TAG, "keypad->box %s: flushing %u held byte(s) unmodified", reason,
           (unsigned) n);
  this->controlbox_uart_->write_array(this->hold_buf_ + this->hold_sent_, n);
  this->hold_sent_ = this->hold_len_;
  // hold_len_ is deliberately left unchanged: keypad_codec_ may still be
  // mid-frame, and hold_buf_[0 .. hold_len_) must keep holding this frame's
  // already-seen bytes (sent or not) so handle_keypad_byte_'s completion
  // logic above can still reason about the whole frame -- in particular so
  // f.bytes[3] is available for the hands-always-win check even when this
  // flush already sent part of the frame. Only hold_sent_ advances here.
}

void DeskLink::service_hold_expiry_() {
  if (!this->override_active_)
    return;
  if ((int32_t) (millis() - this->override_deadline_ms_) >= 0) {
    this->override_active_ = false;
    // Natural expiry ends the hold the same as any other release: whatever
    // ownership status it carried no longer applies once it's gone.
    this->override_owned_by_controller_ = false;
    ESP_LOGI(TAG, "hold expired");
  }
}

void DeskLink::emit_frame_(ctb::Direction dir, const ctb::Frame &f) {
  if (this->client_fd_ < 0)
    return;
  char line[224];
  int n = snprintf(line, sizeof(line), "%u %u %c %c%c ", (unsigned) ++this->seq_,
                   (unsigned) millis(),
                   dir == ctb::Direction::KEYPAD_TO_BOX ? 'K' : 'B',
                   f.crc_valid ? '.' : 'C', f.length_known ? '.' : 'U');
  for (uint8_t i = 0; i < f.len && n < (int) sizeof(line) - 4; i++)
    n += snprintf(line + n, sizeof(line) - n, "%02X", f.bytes[i]);
  if (n < (int) sizeof(line) - 1)
    line[n++] = '\n';
  this->capture_write_(line, (size_t) n);
}

void DeskLink::capture_write_(const char *s, size_t len) {
  if (this->client_fd_ < 0)
    return;
  int sent = ::send(this->client_fd_, s, len, MSG_DONTWAIT);
  if (sent < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Nothing went out, so record framing is still intact. The sequence
      // number gap makes the loss visible to the collector.
      this->drops_++;
      return;
    }
    ESP_LOGW(TAG, "capture client lost on send: errno %d", errno);
    ::close(this->client_fd_);
    this->client_fd_ = -1;
    return;
  }
  if ((size_t) sent < len) {
    // Half a record already went out. Whatever we send next would be
    // concatenated onto it and the collector would parse a glued line.
    // Dropping the client is the only way to keep the stream trustworthy:
    // the capture fails loudly instead of yielding corrupt data.
    ESP_LOGW(TAG, "capture short send (%d of %u), dropping client to preserve framing",
             sent, (unsigned) len);
    ::close(this->client_fd_);
    this->client_fd_ = -1;
  }
}

void DeskLink::service_capture_() {
  if (this->listen_fd_ < 0)
    return;

  if (this->client_fd_ < 0) {
    int fd = ::accept(this->listen_fd_, nullptr, nullptr);
    if (fd < 0)
      return;
    ::fcntl(fd, F_SETFL, O_NONBLOCK);
    int on = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
    this->client_fd_ = fd;
    this->cmd_len_ = 0;
    this->drops_ = 0;
    char hello[80];
    int n = snprintf(hello, sizeof(hello), "# desk_link capture seq=%u\n",
                     (unsigned) this->seq_);
    this->capture_write_(hello, (size_t) n);
    ESP_LOGI(TAG, "capture client connected");
    return;
  }

  char rx[64];
  int n = ::recv(this->client_fd_, rx, sizeof(rx), MSG_DONTWAIT);
  if (n == 0) {
    ESP_LOGI(TAG, "capture client disconnected");
    ::close(this->client_fd_);
    this->client_fd_ = -1;
    return;
  }
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return;  // genuinely nothing pending
    // Anything else (ECONNRESET and friends) means the peer is gone. Without
    // this the single client slot would stay occupied until reboot.
    ESP_LOGW(TAG, "capture client lost on recv: errno %d", errno);
    ::close(this->client_fd_);
    this->client_fd_ = -1;
    return;
  }

  for (int i = 0; i < n; i++) {
    const char c = rx[i];
    if (c == '\n' || c == '\r') {
      if (this->cmd_len_ > 0) {
        this->cmd_buf_[this->cmd_len_] = '\0';
        this->handle_command_(this->cmd_buf_);
        this->cmd_len_ = 0;
      }
    } else if (this->cmd_len_ < sizeof(this->cmd_buf_) - 1) {
      this->cmd_buf_[this->cmd_len_++] = c;
    }
  }
}

void DeskLink::handle_command_(char *line) {
  char verb[16];
  char hex[160];
  // The 4th token is TX/TXC's decimal interval_ms for one verb and HOLD's
  // hex param byte for another, so it cannot be scanned directly as %u: a
  // hex byte containing a letter (e.g. "4A") would stop %u's decimal
  // conversion after the leading digit and leave the letter to be
  // (mis)matched as if it were a whole separate token. Capturing it as a
  // string first and letting each verb below reinterpret it with the base
  // it actually needs -- exactly how `hex` is already reinterpreted per verb
  // -- avoids that silent corruption.
  char param_s[8] = "";
  unsigned count = 1, interval = 38;
  const int fields = sscanf(line, "%15s %159s %u %7s", verb, hex, &count, param_s);
  if (fields < 1)
    return;
  if (fields >= 4)
    interval = (unsigned) strtoul(param_s, nullptr, 10);

  if (strcmp(verb, "STAT") == 0) {
    // Existing fields are kept in place and order -- a workstation script
    // parses this line -- with the two flush counters appended after them.
    // sflush/fflush let that script confirm from outside whether the stall
    // guard (stall_guard_ms_, default 30) is firing during ordinary keypad
    // traffic (it shouldn't be) versus only on genuine keypad silence.
    char out[224];
    int n = snprintf(out, sizeof(out),
                     "# stat seq=%u drops=%u kcrc=%u ksync=%u bcrc=%u bsync=%u "
                     "sflush=%u fflush=%u\n",
                     (unsigned) this->seq_, (unsigned) this->drops_,
                     (unsigned) this->keypad_codec_.crc_errors(),
                     (unsigned) this->keypad_codec_.resyncs(),
                     (unsigned) this->box_codec_.crc_errors(),
                     (unsigned) this->box_codec_.resyncs(),
                     (unsigned) this->stall_flushes_,
                     (unsigned) this->full_buffer_flushes_);
    this->capture_write_(out, (size_t) n);
    return;
  }

  // RELEASE takes no arguments; HOLD 0 is the same request spelled with an
  // explicit duration of zero. Both cancel any active override.
  if (strcmp(verb, "RELEASE") == 0) {
    this->set_hold(0, 0x00, 0);
    this->capture_write_("# release\n", 10);
    return;
  }

  if (strcmp(verb, "PRESET") == 0 || strcmp(verb, "STOREPRESET") == 0) {
    // hex is only guaranteed populated (and null-terminated) once sscanf has
    // actually matched it; without this guard a bare "PRESET" with no
    // number would read strtoul's argument out of an uninitialized buffer.
    const unsigned n = (fields >= 2) ? strtoul(hex, nullptr, 10) : 0;
    if (n < 1 || n > 4) {
      this->capture_write_("# err preset must be 1-4\n", 25);
      return;
    }
    const uint8_t index = (uint8_t) (n - 1);   // zero-based on the wire
    if (verb[0] == 'P') {
      this->recall_preset(index);
    } else {
      this->store_preset(index);
    }
    char out[64];
    int n2 = snprintf(out, sizeof(out), "# %s %u\n", verb, n);
    this->capture_write_(out, (size_t) n2);
    return;
  }

  if (strcmp(verb, "HOLD") == 0) {
    if (fields < 3) {
      this->capture_write_("# hold error=usage HOLD <cmdbyte> <duration_ms>\n", 49);
      return;
    }
    const uint8_t cmdbyte = (uint8_t) strtoul(hex, nullptr, 16);
    // count carries duration_ms here: HOLD shares the same "verb hex u u"
    // sscanf shape as TX/TXC above, and duration_ms is the first %u field.
    uint32_t duration_ms = count;
    // The 4th token, if present, is the param byte in hex, defaulting to
    // 0x00 (plain command-byte hold, today's only behaviour) when absent.
    const uint8_t param = (fields >= 4) ? (uint8_t) strtoul(param_s, nullptr, 16) : 0x00;
    this->set_hold(cmdbyte, param, duration_ms);
    char out[96];
    int n = snprintf(out, sizeof(out), "# hold cmd=%02X param=%02X duration=%u\n", cmdbyte,
                     param, (unsigned) duration_ms);
    this->capture_write_(out, (size_t) n);
    return;
  }

  const bool append_crc = (strcmp(verb, "TXC") == 0);
  if (!append_crc && strcmp(verb, "TX") != 0)
    return;
  if (fields < 2)
    return;

  const size_t hexlen = strlen(hex);
  if (hexlen == 0 || hexlen % 2 != 0)
    return;

  uint8_t bytes[ctb::MAX_FRAME_LEN];
  size_t len = 0;
  const size_t max = append_crc ? (size_t) (ctb::MAX_FRAME_LEN - 2)
                                : (size_t) ctb::MAX_FRAME_LEN;
  for (size_t i = 0; i + 1 < hexlen && len < max; i += 2) {
    const char pair[3] = {hex[i], hex[i + 1], '\0'};
    bytes[len++] = (uint8_t) strtoul(pair, nullptr, 16);
  }
  if (len == 0)
    return;
  if (append_crc) {
    ctb::crc_append(bytes, len);
    len += 2;
  }

  // Clamp rather than silently wrap: inject() takes uint16_t and a hardware
  // operator should see the value that will really be used.
  if (count > 0xFFFF)
    count = 0xFFFF;
  if (interval > 0xFFFF)
    interval = 0xFFFF;

  this->inject(bytes, (uint8_t) len, (uint16_t) count, (uint16_t) interval);

  char out[96];
  int n = snprintf(out, sizeof(out), "# tx len=%u count=%u interval=%u\n",
                   (unsigned) len, count, interval);
  this->capture_write_(out, (size_t) n);
}

void DeskLink::inject(const uint8_t *data, uint8_t len, uint16_t count,
                      uint16_t interval_ms) {
  if (len == 0 || len > ctb::MAX_FRAME_LEN) {
    ESP_LOGW(TAG, "inject rejected: length %u", len);
    return;
  }
  memcpy(this->inject_buf_, data, len);
  this->inject_len_ = len;
  this->inject_remaining_ = count;
  this->inject_interval_ms_ = interval_ms;
  this->inject_next_ms_ = millis();
}

void DeskLink::service_injection_() {
  if (this->inject_remaining_ == 0)
    return;
  const uint32_t now = millis();
  // Signed difference so millis() rollover is handled correctly.
  if ((int32_t) (now - this->inject_next_ms_) < 0)
    return;

  this->controlbox_uart_->write_array(this->inject_buf_, this->inject_len_);
  this->inject_remaining_--;
  // The timer is armed only here, immediately after a send, and nothing else
  // touches it. This is the discipline the old desk_keypad component lacked:
  // it re-armed the timer from the control loop on every pass, so the
  // interval could never elapse and the frame was almost never transmitted.
  this->inject_next_ms_ = now + this->inject_interval_ms_;
}

void DeskLink::set_hold(uint8_t cmd, uint8_t param, uint32_t duration_ms) {
  // Unconditionally cleared here, at the very start, regardless of which
  // branch below runs or why this was called. set_hold() is the single
  // choke point every hold passes through -- the controller dispatch
  // (handle_box_byte_'s UP/DOWN cases), every capture-socket verb
  // (HOLD/RELEASE/PRESET/STOREPRESET, via handle_command_), and
  // recall_preset()/store_preset() -- so clearing it here first, rather than
  // remembering to clear it at each of those call sites individually, is
  // what guarantees override_owned_by_controller_ can only ever be true in
  // the narrow window right after handle_box_byte_'s UP/DOWN cases run: they
  // are the only callers that re-set it to true immediately after this
  // function returns.
  this->override_owned_by_controller_ = false;

  // Capped so a dropped capture connection can never leave the desk driving
  // indefinitely: this is the only path that can end a hold besides RELEASE,
  // a real keypress (hands always win, see handle_keypad_byte_), or
  // service_hold_expiry_ noticing the deadline has passed.
  static const uint32_t MAX_HOLD_MS = 30000;
  if (duration_ms > MAX_HOLD_MS)
    duration_ms = MAX_HOLD_MS;

  if (duration_ms == 0) {
    this->override_active_ = false;
    ESP_LOGI(TAG, "hold released");
    return;
  }
  // During a feed-forward move the controller dispatch in handle_box_byte_
  // re-issues the same UP/DOWN hold on every box frame (~42 ms) purely to
  // push the deadline out. Logging each of those refreshes produced ~24 INFO
  // lines a second for the whole move, formatted and written to the log
  // UART and API log stream from inside the forwarding loop, exactly while
  // timing matters most. Only a hold that actually changes is logged.
  const bool refresh = this->override_active_ && this->override_cmd_ == cmd &&
                       this->override_param_ == param;
  this->override_cmd_ = cmd;
  this->override_param_ = param;
  // Assign the existing expiry field that service_hold_expiry_ already
  // checks, rather than adding a parallel one.
  this->override_deadline_ms_ = millis() + duration_ms;
  this->override_active_ = true;
  if (!refresh)
    ESP_LOGI(TAG, "hold cmd=0x%02X param=0x%02X duration=%u ms", cmd, param,
             (unsigned) duration_ms);
}

void DeskLink::recall_preset(uint8_t index) {
  if (index > 3) {
    ESP_LOGW(TAG, "preset index %u out of range", index);
    return;
  }
  // 0x48 = base 0x40 | preset bit 0x08. A 400 ms hold is comfortably longer
  // than the 41 ms cadence, so the box sees a real press rather than a
  // bounce, and it then positions the desk itself.
  this->set_hold(0x48, index, 400);
}

void DeskLink::store_preset(uint8_t index) {
  if (index > 3) {
    ESP_LOGW(TAG, "preset index %u out of range", index);
    return;
  }
  // 0x44 = base 0x40 | store bit 0x04. Stores the CURRENT height; the
  // protocol has no way to store an arbitrary one. Mirrors the STOREPRESET
  // capture-socket verb in handle_command_() exactly (same cmd/param/
  // duration), just reachable as a direct call instead of only over the
  // socket.
  this->set_hold(0x44, index, 400);
}

void DeskLink::set_preset_height(uint8_t index, float height_cm) {
  if (index < 4)
    this->preset_heights_[index] = height_cm;
}

void DeskLink::move_to(float target_cm) {
  // A new request always supersedes an old one cleanly rather than racing
  // it: if a feed-forward excursion is already under way, abort it before
  // deciding whether this new target is a preset match or another
  // feed-forward move. Without this, a feed-forward move already in
  // progress could still be issuing UP/DOWN holds (see handle_box_byte_'s
  // command() dispatch) at the same time a freshly recalled preset starts
  // driving the box itself, or a second begin_move() could silently
  // re-target an excursion mid-flight in ways MotionController's own
  // one-correction-per-move bookkeeping was never designed to see. This was
  // unreachable before this task -- move_to() had no caller yet -- so it
  // could not previously bite; the cover and the target height number both
  // call in here now, so it can.
  if (this->controller_.state() == ctb::MoveState::MOVING ||
      this->controller_.state() == ctb::MoveState::SETTLING ||
      this->controller_.state() == ctb::MoveState::WAKING) {
    this->controller_.abort(millis());
  }

  // Prefer the control box's own positioning when the target is one of the
  // stored presets: it runs a factory deceleration ramp and stops exactly on
  // target, which feed-forward cannot match. preset_heights_[i] == 0 means
  // that preset's height has never been recorded, so it is skipped rather
  // than treated as a real match at 0 cm.
  // First match wins, in ascending preset index. If two presets are closer
  // together than 2*PRESET_MATCH_CM (observed on this hardware: presets 1 and
  // 4 are 0.5 cm apart against a 0.6 cm match radius), a target in the overlap
  // deterministically resolves to the lower-indexed preset rather than the
  // nearer one. Acceptable: the two candidates are already within the match
  // radius of the requested target, so either is "close enough" by definition.
  for (uint8_t i = 0; i < 4; i++) {
    if (this->preset_heights_[i] > 0 &&
        fabsf(this->preset_heights_[i] - target_cm) <= PRESET_MATCH_CM) {
      ESP_LOGI(TAG, "target %.1f matches preset %u; delegating to the box",
               target_cm, (unsigned) (i + 1));
      this->recall_preset(i);
      return;
    }
  }
  this->controller_.begin_move(target_cm, millis());
}

const char *DeskLink::fault_reason_string() const {
  switch (this->controller_.fault()) {
    case ctb::FaultReason::NONE: return "none";
    case ctb::FaultReason::STALL: return "stall";
    case ctb::FaultReason::TIMEOUT: return "timeout";
    case ctb::FaultReason::NO_HEIGHT: return "no_height";
    case ctb::FaultReason::OUT_OF_RANGE: return "out_of_range";
    case ctb::FaultReason::ABORTED: return "aborted";
  }
  // Every FaultReason enumerator is handled above; this is unreachable
  // unless the enum grows without a matching case being added here, which a
  // -Wswitch warning on the switch above should already catch at compile
  // time. Kept as an explicit, greppable fallback rather than relying on
  // undefined behaviour for a value outside the enum's defined range.
  return "unknown";
}

void DeskLink::dump_config() {
  ESP_LOGCONFIG(TAG, "Desk Link:");
  ESP_LOGCONFIG(TAG, "  Capture port: %u", this->capture_port_);
  ESP_LOGCONFIG(TAG, "  Max frame length: %u", ctb::MAX_FRAME_LEN);
  ESP_LOGCONFIG(TAG, "  Stall guard: %u ms", (unsigned) this->stall_guard_ms_);
}

}  // namespace desk_link
}  // namespace esphome
