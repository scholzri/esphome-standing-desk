#include "desk_link.h"
#include "ctb_crc.h"
#include "esphome/core/log.h"

#include <errno.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace esphome {
namespace desk_link {

static const char *const TAG = "desk_link";

void DeskLink::setup() {
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
  uint8_t b;
  while (this->keypad_uart_->available()) {
    this->keypad_uart_->read_byte(&b);
    if (!this->suppress_keypad_)
      this->controlbox_uart_->write_byte(b);
    this->handle_byte_(ctb::Direction::KEYPAD_TO_BOX, b);
  }
  while (this->controlbox_uart_->available()) {
    this->controlbox_uart_->read_byte(&b);
    // The box stream is always forwarded, even during injection, so the
    // keypad display never goes dark.
    this->keypad_uart_->write_byte(b);
    this->handle_byte_(ctb::Direction::BOX_TO_KEYPAD, b);
  }
  this->service_injection_();
  this->service_capture_();
}

void DeskLink::handle_byte_(ctb::Direction dir, uint8_t b) {
  ctb::Frame f;
  ctb::FrameCodec &codec =
      (dir == ctb::Direction::KEYPAD_TO_BOX) ? this->keypad_codec_ : this->box_codec_;
  if (codec.put(b, &f))
    this->emit_frame_(dir, f);
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
  char verb[8];
  char hex[160];
  unsigned count = 1, interval = 38;
  const int fields = sscanf(line, "%7s %159s %u %u", verb, hex, &count, &interval);
  if (fields < 1)
    return;

  if (strcmp(verb, "STAT") == 0) {
    char out[160];
    int n = snprintf(out, sizeof(out),
                     "# stat seq=%u drops=%u kcrc=%u ksync=%u bcrc=%u bsync=%u\n",
                     (unsigned) this->seq_, (unsigned) this->drops_,
                     (unsigned) this->keypad_codec_.crc_errors(),
                     (unsigned) this->keypad_codec_.resyncs(),
                     (unsigned) this->box_codec_.crc_errors(),
                     (unsigned) this->box_codec_.resyncs());
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
  this->suppress_keypad_ = (count > 0);
}

void DeskLink::service_injection_() {
  if (this->inject_remaining_ == 0) {
    this->suppress_keypad_ = false;
    return;
  }
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

void DeskLink::dump_config() {
  ESP_LOGCONFIG(TAG, "Desk Link:");
  ESP_LOGCONFIG(TAG, "  Capture port: %u", this->capture_port_);
  ESP_LOGCONFIG(TAG, "  Max frame length: %u", ctb::MAX_FRAME_LEN);
}

}  // namespace desk_link
}  // namespace esphome
