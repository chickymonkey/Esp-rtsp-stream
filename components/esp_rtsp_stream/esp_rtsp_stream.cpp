#include "esp_rtsp_stream.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include <esp_heap_caps.h>

#include "esphome/components/network/util.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifndef USE_ESP_RTSP_STREAM_HW_JPEG
#include "esp_jpeg_dec.h"
#endif

namespace esphome {
namespace esp_rtsp_stream {

static const char *const TAG = "esp_rtsp_stream";

#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
static const char *const DECODER_NAME = "ESP32-P4 hardware JPEG + PPA scaler";
#else
static const char *const DECODER_NAME = "esp_new_jpeg (software, SIMD on ESP32-S3)";
#endif

/// RTP channel from a "Transport: ...;interleaved=a-b" header.
static uint8_t interleaved_channel(const std::string &transport, uint8_t fallback) {
  const size_t pos = transport.find("interleaved=");
  return pos == std::string::npos ? fallback : static_cast<uint8_t>(atoi(transport.c_str() + pos + 12));
}

void RtspMjpeg::setup() {
  const char *error = "";
  if (!parse_rtsp_url(this->url_, this->parsed_url_, &error)) {
    ESP_LOGE(TAG, "Invalid URL: %s", error);
    this->mark_failed();
    return;
  }
  if (!this->username_.empty()) {
    this->parsed_url_.username = this->username_;
    this->parsed_url_.password = this->password_;
  }
  this->conn_.set_credentials(this->parsed_url_.username, this->parsed_url_.password);

  for (auto &buf : this->jpeg_buffers_) {
    buf = this->alloc_jpeg_buffer_(this->max_frame_size_);
    if (buf == nullptr) {
      ESP_LOGE(TAG, "Cannot allocate %u byte JPEG buffers; is PSRAM enabled?", (unsigned) this->max_frame_size_);
      this->mark_failed();
      return;
    }
  }

#ifdef USE_ESP_RTSP_STREAM_AUDIO
  if (this->speaker_ != nullptr) {
    // Jitter buffer between the network task (writer) and the main loop (reader), in PSRAM.
    this->audio_ring_storage_ = static_cast<uint8_t *>(heap_caps_malloc(AUDIO_RING_SIZE + 1, MALLOC_CAP_SPIRAM));
    if (this->audio_ring_storage_ != nullptr) {
      this->audio_ring_ = xStreamBufferCreateStatic(AUDIO_RING_SIZE, 1, this->audio_ring_storage_,
                                                    &this->audio_ring_struct_);
    }
    if (this->audio_ring_ == nullptr) {
      ESP_LOGE(TAG, "Cannot allocate the audio buffer; audio disabled");
      this->speaker_ = nullptr;
    }
  }
#endif

  if (xTaskCreatePinnedToCore(decode_task_, "rtsp_dec", 8192, this, 1, &this->decode_task_handle_,
                              this->decode_core_) != pdPASS ||
      xTaskCreatePinnedToCore(network_task_, "rtsp_net", 8192, this, 5, &this->network_task_handle_,
                              this->network_core_) != pdPASS) {
    ESP_LOGE(TAG, "Cannot start tasks");
    this->mark_failed();
    return;
  }
  this->last_stats_ms_ = millis();
}

void RtspMjpeg::loop() {
  const bool new_frame = this->new_frame_.exchange(false);
#ifdef USE_DISPLAY
  if (new_frame && this->display_ != nullptr)
    this->display_->update();
#endif
#ifdef USE_LVGL
  // Runs in the main loop, so touching LVGL objects here is safe.
  if (this->lvgl_canvas_ != nullptr) {
    lv_obj_t *canvas = *this->lvgl_canvas_;
    const bool visible = canvas != nullptr && lv_obj_is_visible(canvas);
    if (visible && this->box_width_ == 0) {
      // No output size configured: fit the frame to the canvas.
      this->box_width_ = lv_obj_get_width(canvas);
      this->box_height_ = lv_obj_get_height(canvas);
    }
    this->frames_wanted_ = visible;
    if (visible && new_frame)
      this->update_lvgl_canvas_(canvas);
  }
#endif
#ifdef USE_ESP_RTSP_STREAM_AUDIO
  this->pump_audio_();
#endif

  const uint32_t now = millis();
  if (this->log_frame_info_ && now - this->last_stats_ms_ >= 30000) {
    const uint32_t decoded = this->decoded_frames_;
    const float fps = (decoded - this->last_stats_decoded_) * 1000.0f / (now - this->last_stats_ms_);
    if (this->streaming_) {
      ESP_LOGI(TAG, "%dx%d stream -> %dx%d, %.1f fps shown, decode %u ms, %u dropped, %u corrupt", this->stream_width_,
               this->stream_height_, this->front_width_, this->front_height_, fps,
               (unsigned) this->last_decode_ms_.load(), (unsigned) this->dropped_frames_.load(),
               (unsigned) this->corrupt_frames_.load());
    }
    this->last_stats_ms_ = now;
    this->last_stats_decoded_ = decoded;
  }
}

void RtspMjpeg::dump_config() {
  ESP_LOGCONFIG(TAG, "RTSP MJPEG stream:");
  ESP_LOGCONFIG(TAG, "  URL: %s", this->parsed_url_.uri.c_str());
  ESP_LOGCONFIG(TAG, "  Authentication: %s", this->parsed_url_.username.empty() ? "none" : "Basic/Digest");
  ESP_LOGCONFIG(TAG, "  Decoder: %s", DECODER_NAME);
  ESP_LOGCONFIG(TAG, "  Max FPS: %d", this->max_fps_);
  if (this->box_width_ > 0)
    ESP_LOGCONFIG(TAG, "  Fit into: %dx%d", this->box_width_, this->box_height_);
  ESP_LOGCONFIG(TAG, "  Rotation: %d", this->rotation_);
  ESP_LOGCONFIG(TAG, "  Max frame size: %u bytes", (unsigned) this->max_frame_size_);
  ESP_LOGCONFIG(TAG, "  Tasks: network on core %d, decode on core %d", this->network_core_, this->decode_core_);
#ifdef USE_ESP_RTSP_STREAM_AUDIO
  if (this->speaker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Audio: G.711/L16 to speaker, %u ms buffer, %s", (unsigned) this->audio_buffer_ms_,
                  this->audio_enabled_ ? "enabled" : "disabled");
  }
#endif
}

// ---------------------------------------------------------------------------------------------------------------
// Network task

void RtspMjpeg::network_task_(void *arg) { static_cast<RtspMjpeg *>(arg)->network_loop_(); }

void RtspMjpeg::network_loop_() {
  uint32_t backoff_ms = 1000;
  while (true) {
    if (!network::is_connected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    const bool got_frames = this->run_session_();
    this->conn_.close();
    this->assembler_.abandon();
    this->streaming_ = false;
    backoff_ms = got_frames ? 1000 : std::min<uint32_t>(backoff_ms * 2, 30000);
    ESP_LOGW(TAG, "Stream stopped, reconnecting in %u s", (unsigned) (backoff_ms / 1000));
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
  }
}

bool RtspMjpeg::run_session_() {
  const RtspUrl &url = this->parsed_url_;
  RtspResponse resp;

  if (!this->conn_.connect(url.host, url.port, 5000))
    return false;
  if (!this->conn_.request("OPTIONS", url.uri, "", resp))
    return false;

  if (!this->conn_.request("DESCRIBE", url.uri, "Accept: application/sdp\r\n", resp))
    return false;
  if (resp.status == 401) {
    ESP_LOGE(TAG, "Camera rejected the credentials (401 Unauthorized)");
    return false;
  }
  if (resp.status != 200) {
    ESP_LOGE(TAG, "DESCRIBE failed with %d; check the stream path in the URL", resp.status);
    return false;
  }

  SdpDescription sdp;
  if (!parse_sdp(resp.body, sdp)) {
    ESP_LOGE(TAG, "Camera offers no video track");
    return false;
  }
  if (strcasecmp(sdp.video.encoding.c_str(), "JPEG") != 0) {
    ESP_LOGE(TAG, "Stream is %s, not MJPEG. Set this camera stream's encoding to MJPEG (usually the sub-stream)",
             sdp.video.encoding.empty() ? "an unknown codec" : sdp.video.encoding.c_str());
    return false;
  }
  this->video_payload_type_ = sdp.video.payload_type;

  std::string base = resp.header("Content-Base");
  if (base.empty())
    base = resp.header("Content-Location");
  if (base.empty())
    base = url.uri;
  const std::string play_uri = resolve_control_url(base, sdp.session_control);

  if (!this->conn_.request("SETUP", resolve_control_url(base, sdp.video.control),
                           "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n", resp))
    return false;
  if (resp.status != 200) {
    ESP_LOGE(TAG, "SETUP failed with %d; the camera may not allow RTP over TCP", resp.status);
    return false;
  }

  std::string session = resp.header("Session");
  uint32_t timeout_s = 60;
  const size_t semi = session.find(';');
  if (semi != std::string::npos) {
    const size_t t = session.find("timeout=", semi);
    if (t != std::string::npos)
      timeout_s = std::max(10, atoi(session.c_str() + t + 8));
    session.resize(semi);
  }
  this->rtp_channel_ = interleaved_channel(resp.header("Transport"), 0);
  this->conn_.set_session(session);

#ifdef USE_ESP_RTSP_STREAM_AUDIO
  if (!this->setup_audio_(sdp.audio, base))
    return false;
#endif

  if (!this->conn_.request("PLAY", play_uri, "Range: npt=0.000-\r\n", resp))
    return false;
  if (resp.status != 200) {
    ESP_LOGE(TAG, "PLAY failed with %d", resp.status);
    return false;
  }

  ESP_LOGI(TAG, "Streaming MJPEG from %s", url.host.c_str());
  this->streaming_ = true;
  this->have_seq_ = false;
  this->session_frames_ = 0;
  this->assembler_.set_buffer(this->jpeg_buffers_[this->fill_index_], this->max_frame_size_);

  // Cameras drop sessions that stay silent for `timeout`; OPTIONS refreshes it on Hikvision and Dahua alike.
  const uint32_t keepalive_ms = timeout_s * 1000 / 2;
  uint32_t last_keepalive_ms = millis();
  size_t garbage = 0;
  while (true) {
    if (millis() - last_keepalive_ms > keepalive_ms) {
      if (!this->conn_.send_only("OPTIONS", url.uri, ""))
        break;
      last_keepalive_ms = millis();
    }

    uint8_t first;
    if (!this->conn_.read_byte(first))
      break;
    if (first == '$') {
      uint8_t hdr[3];
      if (!this->conn_.read_exact(hdr, sizeof(hdr)))
        break;
      const uint16_t len = (hdr[1] << 8) | hdr[2];
      bool ok;
      if (hdr[0] == this->rtp_channel_) {
        ok = this->handle_rtp_(len);
#ifdef USE_ESP_RTSP_STREAM_AUDIO
      } else if (hdr[0] == this->audio_rtp_channel_ && this->audio_codec_ != AudioCodec::NONE) {
        ok = this->handle_audio_rtp_(len);
#endif
      } else {
        ok = this->conn_.skip(len);
      }
      if (!ok)
        break;
      garbage = 0;
    } else if (first == 'R') {
      // Reply to a keepalive. A 401 means the Digest nonce went stale: re-authenticate.
      if (!this->conn_.read_response_tail(first, resp))
        break;
      if (resp.status == 401 && this->conn_.refresh_challenge(resp) && !this->conn_.send_only("OPTIONS", url.uri, ""))
        break;
    } else if (++garbage > 65536) {
      ESP_LOGW(TAG, "Lost sync with the interleaved stream");
      break;
    }
  }
  return this->session_frames_ > 0;
}

bool RtspMjpeg::handle_rtp_(uint16_t len) {
  size_t used = 0;
  // Reads `n` header bytes, or skips the rest of a malformed packet.
  auto take = [&](uint8_t *dst, size_t n, bool &ok) -> bool {
    if (used + n > len) {
      ok = this->conn_.skip(len - used);
      return false;
    }
    used += n;
    ok = dst == nullptr ? this->conn_.skip(n) : this->conn_.read_exact(dst, n);
    return ok;
  };
  bool ok = true;

  uint8_t rtp[12];
  if (!take(rtp, sizeof(rtp), ok))
    return ok;
  const bool padding = rtp[0] & 0x20;
  const bool extension = rtp[0] & 0x10;
  const bool marker = rtp[1] & 0x80;
  const int payload_type = rtp[1] & 0x7F;
  const uint16_t seq = (rtp[2] << 8) | rtp[3];
  if (!take(nullptr, (rtp[0] & 0x0F) * 4, ok))
    return ok;
  if (extension) {
    uint8_t ext[4];
    if (!take(ext, sizeof(ext), ok) || !take(nullptr, ((ext[2] << 8) | ext[3]) * 4, ok))
      return ok;
  }
  if (payload_type != this->video_payload_type_)
    return this->conn_.skip(len - used);

  if (this->have_seq_ && seq != static_cast<uint16_t>(this->last_seq_ + 1) && this->assembler_.in_frame()) {
    this->assembler_.abandon();
    this->corrupt_frames_++;
  }
  this->have_seq_ = true;
  this->last_seq_ = seq;

  uint8_t main_hdr[8];
  if (!take(main_hdr, sizeof(main_hdr), ok))
    return ok;
  JpegPayloadHeader hdr;
  hdr.type_specific = main_hdr[0];
  hdr.fragment_offset = (main_hdr[1] << 16) | (main_hdr[2] << 8) | main_hdr[3];
  hdr.type = main_hdr[4];
  hdr.q = main_hdr[5];
  hdr.width = main_hdr[6] * 8;
  hdr.height = main_hdr[7] * 8;

  uint16_t restart_interval = 0;
  if (hdr.type >= 64 && hdr.type < 128) {
    uint8_t restart_hdr[4];
    if (!take(restart_hdr, sizeof(restart_hdr), ok))
      return ok;
    restart_interval = (restart_hdr[0] << 8) | restart_hdr[1];
  }

  size_t qtables_len = 0;
  uint8_t qtables_precision = 0;
  if (hdr.fragment_offset == 0 && hdr.q >= 128) {
    uint8_t quant_hdr[4];
    if (!take(quant_hdr, sizeof(quant_hdr), ok))
      return ok;
    qtables_precision = quant_hdr[1];
    qtables_len = (quant_hdr[2] << 8) | quant_hdr[3];
    if (qtables_len > sizeof(this->qtables_)) {
      this->assembler_.abandon();
      return this->conn_.skip(len - used);
    }
    if (!take(this->qtables_, qtables_len, ok))
      return ok;
  }

  const uint32_t now = millis();
  if (hdr.fragment_offset == 0) {
    if (this->assembler_.in_frame())
      this->corrupt_frames_++;  // the previous frame never got its marker packet
    if (!this->assembler_.begin_frame(hdr, restart_interval, this->qtables_, qtables_len, qtables_precision) &&
        now - this->last_warning_ms_ > 10000) {
      this->last_warning_ms_ = now;
      ESP_LOGW(TAG, "Cannot use frame (type %u, Q %u, %ux%u): %s", hdr.type, hdr.q, hdr.width, hdr.height,
               this->assembler_.last_error());
    }
  }

  const size_t payload = len - used;
  const bool was_in_frame = this->assembler_.in_frame();
  uint8_t *dst = this->assembler_.fragment_target(hdr.fragment_offset, payload);
  if (dst == nullptr) {
    if (was_in_frame) {
      this->corrupt_frames_++;
      if (now - this->last_warning_ms_ > 10000) {
        this->last_warning_ms_ = now;
        ESP_LOGW(TAG, "Dropped frame: %s", this->assembler_.last_error());
      }
    }
    if (!this->conn_.skip(payload))
      return false;
  } else {
    if (!this->conn_.read_exact(dst, payload))
      return false;
    size_t pad = padding && payload > 0 ? dst[payload - 1] : 0;
    this->assembler_.fragment_written(payload - std::min(pad, payload));
  }

  if (marker) {
    const size_t jpeg_len = this->assembler_.end_frame();
    if (jpeg_len > 0)
      this->frame_complete_(jpeg_len);
  }
  return true;
}

void RtspMjpeg::frame_complete_(size_t jpeg_len) {
  this->received_frames_++;
  this->session_frames_++;
  this->stream_width_ = this->assembler_.width();
  this->stream_height_ = this->assembler_.height();
  if (!this->frames_wanted_)
    return;

  // Drop rather than queue: a backlog would show an ever older picture.
  const uint32_t now = millis();
  const uint32_t interval = 1000 / this->max_fps_;
  if (static_cast<int32_t>(now + interval / 4 - this->next_frame_due_ms_) < 0 || this->decoder_busy_) {
    this->dropped_frames_++;
    return;
  }
  this->next_frame_due_ms_ = static_cast<int32_t>(now - this->next_frame_due_ms_) > static_cast<int32_t>(interval)
                                 ? now + interval
                                 : this->next_frame_due_ms_ + interval;

  this->ready_index_ = this->fill_index_;
  this->ready_len_ = jpeg_len;
  this->ready_width_ = this->stream_width_;
  this->ready_height_ = this->stream_height_;
  this->fill_index_ ^= 1;
  this->assembler_.set_buffer(this->jpeg_buffers_[this->fill_index_], this->max_frame_size_);
  this->decoder_busy_ = true;
  xTaskNotifyGive(this->decode_task_handle_);
}

// ---------------------------------------------------------------------------------------------------------------
// Audio

#ifdef USE_ESP_RTSP_STREAM_AUDIO

// ITU-T G.711 expansion (the classic Sun reference implementation).
static int16_t ulaw_to_pcm(uint8_t u) {
  u = ~u;
  int t = ((u & 0x0F) << 3) + 0x84;
  t <<= (u & 0x70) >> 4;
  return static_cast<int16_t>((u & 0x80) ? (0x84 - t) : (t - 0x84));
}

static int16_t alaw_to_pcm(uint8_t a) {
  a ^= 0x55;
  int t = (a & 0x0F) << 4;
  const int seg = (a & 0x70) >> 4;
  if (seg == 0) {
    t += 8;
  } else {
    t += 0x108;
    if (seg > 1)
      t <<= seg - 1;
  }
  return static_cast<int16_t>((a & 0x80) ? t : -t);
}

bool RtspMjpeg::setup_audio_(const SdpTrack &track, const std::string &base) {
  this->audio_codec_ = AudioCodec::NONE;
  if (this->speaker_ == nullptr || track.payload_type < 0)
    return true;

  AudioCodec codec;
  if (strcasecmp(track.encoding.c_str(), "PCMU") == 0) {
    codec = AudioCodec::PCMU;
  } else if (strcasecmp(track.encoding.c_str(), "PCMA") == 0) {
    codec = AudioCodec::PCMA;
  } else if (strcasecmp(track.encoding.c_str(), "L16") == 0) {
    codec = AudioCodec::L16;
  } else {
    ESP_LOGW(TAG, "Camera audio is %s; only G.711 (PCMU/PCMA) and L16 are supported. Set the camera's audio "
                  "encoding to G.711 to hear it",
             track.encoding.empty() ? "an unknown codec" : track.encoding.c_str());
    return true;
  }
  const uint32_t rate = track.clock_rate > 0 ? track.clock_rate : 8000;
  const uint8_t channels = std::min(2, std::max(1, track.channels));

  RtspResponse resp;
  if (!this->conn_.request("SETUP", resolve_control_url(base, track.control),
                           "Transport: RTP/AVP/TCP;unicast;interleaved=2-3\r\n", resp))
    return false;  // transport error: the session is unusable
  if (resp.status != 200) {
    ESP_LOGW(TAG, "Audio SETUP failed with %d; continuing without audio", resp.status);
    return true;
  }
  this->audio_rtp_channel_ = interleaved_channel(resp.header("Transport"), 2);
  this->audio_payload_type_ = track.payload_type;
  if (rate != this->audio_sample_rate_ || channels != this->audio_channels_) {
    this->audio_sample_rate_ = rate;
    this->audio_channels_ = channels;
    this->audio_format_generation_++;  // tells the main loop to restart the speaker with the new format
  }
  this->audio_codec_ = codec;
  ESP_LOGI(TAG, "Audio: %s, %u Hz, %u channel(s)", track.encoding.c_str(), (unsigned) rate, channels);
  return true;
}

bool RtspMjpeg::handle_audio_rtp_(uint16_t len) {
  if (len < 12 || len > sizeof(this->audio_packet_))
    return this->conn_.skip(len);
  if (!this->conn_.read_exact(this->audio_packet_, len))
    return false;
  if (!this->audio_wanted_)
    return true;

  const uint8_t *p = this->audio_packet_;
  size_t start = 12 + (p[0] & 0x0F) * 4;
  if ((p[0] & 0x10) && start + 4 <= len)
    start += 4 + ((p[start + 2] << 8) | p[start + 3]) * 4;
  size_t end = len;
  if (p[0] & 0x20)
    end -= std::min<size_t>(p[len - 1], len);
  if ((p[1] & 0x7F) != this->audio_payload_type_ || start >= end)
    return true;

  // Decode to little-endian 16-bit PCM in chunks. A chunk that does not fit is dropped whole, so the
  // buffer only ever holds complete frames.
  int16_t pcm[256];
  size_t count = 0;
  auto flush = [&]() {
    const size_t bytes = count * sizeof(int16_t);
    if (xStreamBufferSpacesAvailable(this->audio_ring_) >= bytes) {
      xStreamBufferSend(this->audio_ring_, pcm, bytes, 0);
    } else {
      this->audio_overruns_++;
    }
    count = 0;
  };
  if (this->audio_codec_ == AudioCodec::L16) {
    for (size_t i = start; i + 1 < end; i += 2) {
      pcm[count++] = static_cast<int16_t>((p[i] << 8) | p[i + 1]);  // network byte order
      if (count == 256)
        flush();
    }
  } else {
    const bool ulaw = this->audio_codec_ == AudioCodec::PCMU;
    for (size_t i = start; i < end; i++) {
      pcm[count++] = ulaw ? ulaw_to_pcm(p[i]) : alaw_to_pcm(p[i]);
      if (count == 256)
        flush();
    }
  }
  if (count > 0)
    flush();
  return true;
}

void RtspMjpeg::drain_audio_() {
  uint8_t scratch[256];
  while (xStreamBufferReceive(this->audio_ring_, scratch, sizeof(scratch), 0) > 0) {
  }
  this->audio_out_len_ = this->audio_out_pos_ = 0;
}

void RtspMjpeg::pump_audio_() {
  if (this->speaker_ == nullptr || this->audio_ring_ == nullptr)
    return;

  // Audio follows the picture: off when disabled, when the stream is down or while the canvas is hidden.
  const bool wanted = this->audio_enabled_ && this->streaming_ && this->frames_wanted_ &&
                      this->audio_codec_ != AudioCodec::NONE;
  this->audio_wanted_ = wanted;
  const uint32_t generation = this->audio_format_generation_;
  if (!wanted || generation != this->speaker_generation_) {
    if (this->speaker_started_) {
      this->speaker_->stop();
      this->speaker_started_ = false;
    }
    this->drain_audio_();
    this->speaker_generation_ = generation;
    return;
  }

  const size_t frame_bytes = 2 * this->audio_channels_;
  const size_t bytes_per_ms = this->audio_sample_rate_ * frame_bytes / 1000;
  const size_t target_bytes = this->audio_buffer_ms_ * bytes_per_ms;
  const size_t available = xStreamBufferBytesAvailable(this->audio_ring_);

  if (!this->speaker_started_) {
    if (available < target_bytes)
      return;  // pre-buffer to ride out network jitter
    this->speaker_->set_audio_stream_info(audio::AudioStreamInfo(16, this->audio_channels_, this->audio_sample_rate_));
    this->speaker_->start();
    this->speaker_started_ = true;
  }

  // The camera clock and the I2S clock drift apart. If audio piles up, skip ahead to keep the delay bounded.
  if (available > 3 * target_bytes) {
    size_t excess = (available - target_bytes) / frame_bytes * frame_bytes;
    uint8_t scratch[256];
    while (excess > 0) {
      const size_t n = xStreamBufferReceive(this->audio_ring_, scratch, std::min(excess, sizeof(scratch)), 0);
      if (n == 0)
        break;
      excess -= n;
    }
  }

  // Feed whatever the speaker accepts without blocking the main loop.
  const size_t chunk = sizeof(this->audio_out_) / frame_bytes * frame_bytes;
  for (int i = 0; i < 8; i++) {
    if (this->audio_out_pos_ == this->audio_out_len_) {
      this->audio_out_len_ = xStreamBufferReceive(this->audio_ring_, this->audio_out_, chunk, 0);
      this->audio_out_pos_ = 0;
      if (this->audio_out_len_ == 0)
        break;
    }
    const size_t written = this->speaker_->play(this->audio_out_ + this->audio_out_pos_,
                                                this->audio_out_len_ - this->audio_out_pos_, 0);
    this->audio_out_pos_ += written;
    if (written == 0)
      break;
  }
}

#endif

// ---------------------------------------------------------------------------------------------------------------
// Decode task

void RtspMjpeg::decode_task_(void *arg) { static_cast<RtspMjpeg *>(arg)->decode_loop_(); }

void RtspMjpeg::decode_loop_() {
#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
  jpeg_decode_engine_cfg_t engine_cfg = {};
  engine_cfg.intr_priority = 0;
  engine_cfg.timeout_ms = 200;
  ppa_client_config_t ppa_cfg = {};
  ppa_cfg.oper_type = PPA_OPERATION_SRM;
  ppa_cfg.max_pending_trans_num = 1;
  // On failure keep running: decode_frame_() then rejects every frame, which keeps the hand-off consistent.
  if (jpeg_new_decoder_engine(&engine_cfg, &this->hw_decoder_) != ESP_OK)
    ESP_LOGE(TAG, "Cannot initialise the hardware JPEG decoder");
  if (ppa_register_client(&ppa_cfg, &this->ppa_client_) != ESP_OK)
    ESP_LOGE(TAG, "Cannot initialise the PPA");
#endif

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint32_t start = millis();
    const bool ok = this->decode_frame_(this->jpeg_buffers_[this->ready_index_], this->ready_len_, this->ready_width_,
                                        this->ready_height_);
    this->last_decode_ms_ = millis() - start;
    this->decoder_busy_ = false;
    if (!ok) {
      this->corrupt_frames_++;
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(this->frame_mutex_);
      this->front_ ^= 1;
      this->front_width_ = this->back_width_;
      this->front_height_ = this->back_height_;
      this->front_stride_ = this->back_stride_;
    }
    if (++this->decoded_frames_ == 1) {
      ESP_LOGI(TAG, "First frame: %dx%d stream -> %dx%d output, decoded in %u ms", this->ready_width_,
               this->ready_height_, this->back_width_, this->back_height_, (unsigned) this->last_decode_ms_.load());
    }
    this->new_frame_ = true;
  }
}

RtspMjpeg::OutputPlan RtspMjpeg::plan_output_(int src_width, int src_height) const {
  const bool swap = this->rotation_ == 90 || this->rotation_ == 270;
  float scale = 1.0f;
  if (this->box_width_ > 0 && this->box_height_ > 0) {
    // The box is in display orientation; compare it with the source before rotation.
    const int box_w = swap ? this->box_height_ : this->box_width_;
    const int box_h = swap ? this->box_width_ : this->box_height_;
    scale = std::min({1.0f, static_cast<float>(box_w) / src_width, static_cast<float>(box_h) / src_height});
  }

  // The exact aspect-preserving size. The hardware scalers can only get close to it, so they
  // aim at the next size up and resize_rgb565() closes the last few pixels.
  OutputPlan plan;
  plan.target_width = std::max(1, static_cast<int>(src_width * scale + 0.01f));
  plan.target_height = std::max(1, static_cast<int>(src_height * scale + 0.01f));
#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
  // PPA scale factors have 1/16 resolution.
  plan.scale = std::min(1.0f, std::max(1.0f / 16, std::ceil(scale * 16 - 0.01f) / 16));
  plan.scaled_width = static_cast<int>(plan.scale * src_width);
  plan.scaled_height = static_cast<int>(plan.scale * src_height);
#else
  // Only ask esp_new_jpeg for the 1/2, 1/4, 1/8 reductions a JPEG decoder does natively, and only when the
  // result stays a multiple of 8 and no smaller than the target; finish_frame_() does the rest.
  int factor = 1;
  for (int f = 2; f <= 8; f *= 2) {
    if (src_width % (8 * f) == 0 && src_height % (8 * f) == 0 && src_width / f >= plan.target_width &&
        src_height / f >= plan.target_height)
      factor = f;
  }
  plan.scale = scale;
  plan.scaled_width = src_width / factor;
  plan.scaled_height = src_height / factor;
#endif
  plan.stage_width = swap ? plan.scaled_height : plan.scaled_width;
  plan.stage_height = swap ? plan.scaled_width : plan.scaled_height;
  plan.out_width = swap ? plan.target_height : plan.target_width;
  plan.out_height = swap ? plan.target_width : plan.target_height;
  return plan;
}

/// Halves an RGB565 image in place by averaging 2x2 blocks (stride == width).
static void halve_rgb565(uint16_t *buf, int &width, int &height) {
  const int new_w = width / 2, new_h = height / 2;
  for (int y = 0; y < new_h; y++) {
    const uint16_t *row0 = buf + 2 * y * width;
    const uint16_t *row1 = row0 + width;
    uint16_t *dst = buf + y * new_w;  // never ahead of the rows still to be read
    for (int x = 0; x < new_w; x++) {
      const uint32_t a = row0[2 * x], b = row0[2 * x + 1], c = row1[2 * x], d = row1[2 * x + 1];
      const uint32_t r = (((a >> 11) & 0x1F) + ((b >> 11) & 0x1F) + ((c >> 11) & 0x1F) + ((d >> 11) & 0x1F) + 2) >> 2;
      const uint32_t g = (((a >> 5) & 0x3F) + ((b >> 5) & 0x3F) + ((c >> 5) & 0x3F) + ((d >> 5) & 0x3F) + 2) >> 2;
      const uint32_t bl = ((a & 0x1F) + (b & 0x1F) + (c & 0x1F) + (d & 0x1F) + 2) >> 2;
      dst[x] = static_cast<uint16_t>((r << 11) | (g << 5) | bl);
    }
  }
  width = new_w;
  height = new_h;
}

/// Bilinear RGB565 (little-endian) resize. Used after halving, so the ratio is below 2:1.
static void resize_rgb565(const uint16_t *src, int src_w, int src_h, int src_stride, uint16_t *dst, int dst_w,
                          int dst_h) {
  const uint32_t x_step = dst_w > 1 ? (static_cast<uint32_t>(src_w - 1) << 16) / (dst_w - 1) : 0;
  const uint32_t y_step = dst_h > 1 ? (static_cast<uint32_t>(src_h - 1) << 16) / (dst_h - 1) : 0;
  for (int y = 0; y < dst_h; y++) {
    const uint32_t fy = y * y_step;
    const int y0 = fy >> 16;
    const int y1 = std::min(y0 + 1, src_h - 1);
    const uint32_t wy = (fy >> 8) & 0xFF;
    const uint16_t *row0 = src + y0 * src_stride;
    const uint16_t *row1 = src + y1 * src_stride;
    for (int x = 0; x < dst_w; x++) {
      const uint32_t fx = x * x_step;
      const int x0 = fx >> 16;
      const int x1 = std::min(x0 + 1, src_w - 1);
      const uint32_t wx = (fx >> 8) & 0xFF;
      const uint32_t p00 = row0[x0], p01 = row0[x1], p10 = row1[x0], p11 = row1[x1];
      auto mix = [&](int shift, uint32_t mask) -> uint32_t {
        const uint32_t top = ((p00 >> shift) & mask) * (256 - wx) + ((p01 >> shift) & mask) * wx;
        const uint32_t bottom = ((p10 >> shift) & mask) * (256 - wx) + ((p11 >> shift) & mask) * wx;
        return ((top * (256 - wy) + bottom * wy) >> 16) << shift;
      };
      *dst++ = static_cast<uint16_t>(mix(11, 0x1F) | mix(5, 0x3F) | mix(0, 0x1F));
    }
  }
}

uint8_t *RtspMjpeg::alloc_jpeg_buffer_(size_t bytes) {
#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
  jpeg_decode_memory_alloc_cfg_t cfg = {};
  cfg.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER;
  size_t allocated = 0;
  return static_cast<uint8_t *>(jpeg_alloc_decoder_mem(bytes, &cfg, &allocated));
#else
  return static_cast<uint8_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#endif
}

void RtspMjpeg::free_buffer_(uint8_t *buf) { heap_caps_free(buf); }

bool RtspMjpeg::ensure_buffer_(uint8_t *&buf, size_t &capacity, size_t bytes) {
  if (capacity >= bytes)
    return true;
  this->free_buffer_(buf);
  capacity = 0;
#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
  // Cache-line aligned, as the JPEG engine and PPA require.
  jpeg_decode_memory_alloc_cfg_t cfg = {};
  cfg.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER;
  size_t allocated = 0;
  buf = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(bytes, &cfg, &allocated));
#else
  // esp_new_jpeg's SIMD path needs 16-byte aligned output.
  size_t allocated = bytes;
  buf = static_cast<uint8_t *>(heap_caps_aligned_calloc(16, 1, bytes, MALLOC_CAP_SPIRAM));
#endif
  if (buf == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate a %u byte frame buffer", (unsigned) bytes);
    return false;
  }
  capacity = allocated;
  return true;
}

bool RtspMjpeg::finish_frame_(const OutputPlan &plan, int back) {
  if (plan.stage_width != plan.out_width || plan.stage_height != plan.out_height) {
    if (!this->ensure_buffer_(this->rgb_[back], this->rgb_capacity_[back],
                              static_cast<size_t>(plan.out_width) * plan.out_height * 2))
      return false;
    uint16_t *staged = reinterpret_cast<uint16_t *>(this->scaled_);
    int width = plan.stage_width, height = plan.stage_height;
    while (width >= 2 * plan.out_width && height >= 2 * plan.out_height)
      halve_rgb565(staged, width, height);
    resize_rgb565(staged, width, height, width, reinterpret_cast<uint16_t *>(this->rgb_[back]), plan.out_width,
                  plan.out_height);
  }
  this->back_width_ = plan.out_width;
  this->back_height_ = plan.out_height;
  this->back_stride_ = plan.out_width;
  return true;
}

#ifdef USE_ESP_RTSP_STREAM_HW_JPEG

bool RtspMjpeg::decode_frame_(const uint8_t *jpeg, size_t len, int src_width, int src_height) {
  if (this->hw_decoder_ == nullptr || this->ppa_client_ == nullptr)
    return false;
  jpeg_decode_picture_info_t info;
  if (jpeg_decoder_get_info(jpeg, len, &info) != ESP_OK) {
    ESP_LOGD(TAG, "Cannot parse JPEG header");
    return false;
  }
  src_width = info.width;
  src_height = info.height;

  // The decoder writes whole MCUs, so rows are padded to the MCU size.
  const int mcu_w = (info.sample_method == JPEG_DOWN_SAMPLING_YUV444 || info.sample_method == JPEG_DOWN_SAMPLING_GRAY)
                        ? 8
                        : 16;
  const int mcu_h = info.sample_method == JPEG_DOWN_SAMPLING_YUV420 ? 16 : 8;
  const int padded_w = (src_width + mcu_w - 1) / mcu_w * mcu_w;
  const int padded_h = (src_height + mcu_h - 1) / mcu_h * mcu_h;
  const size_t decoded_bytes = static_cast<size_t>(padded_w) * padded_h * 2;

  const OutputPlan plan = this->plan_output_(src_width, src_height);
  const int back = this->front_ ^ 1;
  const bool direct = this->rotation_ == 0 && plan.target_width == src_width && plan.target_height == src_height;

  uint8_t *&target = direct ? this->rgb_[back] : this->staging_;
  size_t &target_capacity = direct ? this->rgb_capacity_[back] : this->staging_capacity_;
  if (!this->ensure_buffer_(target, target_capacity, decoded_bytes))
    return false;

  jpeg_decode_cfg_t cfg = {};
  cfg.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  cfg.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR;  // little-endian RGB565
  cfg.conv_std = JPEG_YUV_RGB_CONV_STD_BT601;
  uint32_t written = 0;
  esp_err_t err = jpeg_decoder_process(this->hw_decoder_, &cfg, jpeg, len, target, target_capacity, &written);
  if (err != ESP_OK) {
    ESP_LOGD(TAG, "Hardware decode failed: %s", esp_err_to_name(err));
    return false;
  }

  if (direct) {
    this->back_width_ = src_width;
    this->back_height_ = src_height;
    this->back_stride_ = padded_w;
    return true;
  }

  // PPA writes straight into the frame buffer when it hits the exact size, else into scaled_ for a final resize.
  const bool exact = plan.stage_width == plan.out_width && plan.stage_height == plan.out_height;
  uint8_t *&ppa_out = exact ? this->rgb_[back] : this->scaled_;
  size_t &ppa_out_capacity = exact ? this->rgb_capacity_[back] : this->scaled_capacity_;
  if (!this->ensure_buffer_(ppa_out, ppa_out_capacity, static_cast<size_t>(plan.stage_width) * plan.stage_height * 2))
    return false;

  ppa_srm_oper_config_t op = {};
  op.in.buffer = this->staging_;
  op.in.pic_w = padded_w;
  op.in.pic_h = padded_h;
  op.in.block_w = src_width;
  op.in.block_h = src_height;
  op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  op.out.buffer = ppa_out;
  op.out.buffer_size = ppa_out_capacity;
  op.out.pic_w = plan.stage_width;
  op.out.pic_h = plan.stage_height;
  op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  // PPA rotates counter-clockwise; `rotation` is clockwise.
  op.rotation_angle = this->rotation_ == 90    ? PPA_SRM_ROTATION_ANGLE_270
                      : this->rotation_ == 180 ? PPA_SRM_ROTATION_ANGLE_180
                      : this->rotation_ == 270 ? PPA_SRM_ROTATION_ANGLE_90
                                               : PPA_SRM_ROTATION_ANGLE_0;
  op.scale_x = plan.scale;
  op.scale_y = plan.scale;
  op.mode = PPA_TRANS_MODE_BLOCKING;
  err = ppa_do_scale_rotate_mirror(this->ppa_client_, &op);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PPA scale/rotate failed: %s", esp_err_to_name(err));
    return false;
  }
  return this->finish_frame_(plan, back);
}

#else

bool RtspMjpeg::decode_frame_(const uint8_t *jpeg, size_t len, int src_width, int src_height) {
  const int back = this->front_ ^ 1;
  // The RTP header gives the size up front; a pass-through JPEG might differ, so retry once with the real size.
  for (int attempt = 0; attempt < 2; attempt++) {
    OutputPlan plan = this->plan_output_(src_width, src_height);
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    if (plan.scaled_width != src_width || plan.scaled_height != src_height) {
      cfg.scale.width = plan.scaled_width;
      cfg.scale.height = plan.scaled_height;
    }
    cfg.rotate = this->rotation_ == 90    ? JPEG_ROTATE_90D
                 : this->rotation_ == 180 ? JPEG_ROTATE_180D
                 : this->rotation_ == 270 ? JPEG_ROTATE_270D
                                          : JPEG_ROTATE_0D;

    jpeg_dec_handle_t decoder = nullptr;
    if (jpeg_dec_open(&cfg, &decoder) != JPEG_ERR_OK)
      return false;
    jpeg_dec_io_t io = {};
    io.inbuf = const_cast<uint8_t *>(jpeg);
    io.inbuf_len = len;
    jpeg_dec_header_info_t info;
    jpeg_error_t err = jpeg_dec_parse_header(decoder, &io, &info);
    if (err == JPEG_ERR_OK && (info.width != src_width || info.height != src_height) && attempt == 0) {
      jpeg_dec_close(decoder);
      src_width = info.width;
      src_height = info.height;
      continue;
    }

    int out_len = 0;
    if (err == JPEG_ERR_OK)
      err = jpeg_dec_get_outbuf_len(decoder, &out_len);
    // Trust the decoder's output size, not our request: it may decline a scale and decode at full size.
    const bool scaling = plan.scaled_width != src_width || plan.scaled_height != src_height;
    if (err == JPEG_ERR_OK && out_len < plan.scaled_width * plan.scaled_height * 2) {
      ESP_LOGW(TAG, "Unexpected decoder output size %d for %dx%d", out_len, src_width, src_height);
      err = JPEG_ERR_FAIL;
    }
    if (err == JPEG_ERR_OK && scaling && out_len >= src_width * src_height * 2) {
      if (!this->scale_fallback_logged_) {
        this->scale_fallback_logged_ = true;
        ESP_LOGD(TAG, "Decoder ignored scale %dx%d, scaling %dx%d in software", plan.scaled_width,
                 plan.scaled_height, src_width, src_height);
      }
      const bool swap = this->rotation_ == 90 || this->rotation_ == 270;
      plan.scaled_width = src_width;
      plan.scaled_height = src_height;
      plan.stage_width = swap ? src_height : src_width;
      plan.stage_height = swap ? src_width : src_height;
    }

    // Decode straight into the frame buffer when the decoder hits the exact size, else into scaled_.
    const bool exact = plan.stage_width == plan.out_width && plan.stage_height == plan.out_height;
    uint8_t *&out = exact ? this->rgb_[back] : this->scaled_;
    size_t &out_capacity = exact ? this->rgb_capacity_[back] : this->scaled_capacity_;
    if (err == JPEG_ERR_OK && !this->ensure_buffer_(out, out_capacity, out_len))
      err = JPEG_ERR_NO_MEM;
    if (err == JPEG_ERR_OK) {
      io.outbuf = out;
      err = jpeg_dec_process(decoder, &io);
    }
    jpeg_dec_close(decoder);
    if (err != JPEG_ERR_OK) {
      ESP_LOGD(TAG, "Decode failed: %d", err);
      return false;
    }
    return this->finish_frame_(plan, back);
  }
  return false;
}

#endif

// ---------------------------------------------------------------------------------------------------------------
// Drawing

#ifdef USE_DISPLAY
void RtspMjpeg::draw(display::Display &it) {
  int width, height;
  {
    std::lock_guard<std::mutex> lock(this->frame_mutex_);
    width = this->front_width_;
    height = this->front_height_;
  }
  this->draw(it, (it.get_width() - width) / 2, (it.get_height() - height) / 2);
}

void RtspMjpeg::draw(display::Display &it, int x, int y) {
  std::lock_guard<std::mutex> lock(this->frame_mutex_);
  if (this->front_width_ == 0)
    return;
  // The ColorOrder overload exists in every current ESPHome release; newer ones only deprecate it.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  it.draw_pixels_at(x, y, this->front_width_, this->front_height_, this->rgb_[this->front_], display::COLOR_ORDER_RGB,
                    display::COLOR_BITNESS_565, false, 0, 0, this->front_stride_ - this->front_width_);
#pragma GCC diagnostic pop
}
#endif

#ifdef USE_LVGL
void RtspMjpeg::update_lvgl_canvas_(lv_obj_t *canvas) {
  lv_draw_buf_t *buf = lv_canvas_get_draw_buf(canvas);
  if (buf == nullptr || buf->data == nullptr || buf->header.cf != LV_COLOR_FORMAT_RGB565) {
    ESP_LOGW(TAG, "LVGL canvas must be an RGB565 canvas (color_depth 16, transparent: false)");
    this->lvgl_canvas_ = nullptr;
    return;
  }
  const int dst_w = buf->header.w;
  const int dst_h = buf->header.h;
  const int dst_stride = buf->header.stride;

  std::lock_guard<std::mutex> lock(this->frame_mutex_);
  if (this->front_width_ == 0)
    return;
  if (this->front_width_ != this->canvas_frame_width_ || this->front_height_ != this->canvas_frame_height_) {
    memset(buf->data, 0, static_cast<size_t>(dst_stride) * dst_h);  // black borders around a smaller frame
    this->canvas_frame_width_ = this->front_width_;
    this->canvas_frame_height_ = this->front_height_;
  }
  // Centre the frame; crop it if it is larger than the canvas.
  const int w = std::min(this->front_width_, dst_w);
  const int h = std::min(this->front_height_, dst_h);
  const int dst_x = (dst_w - w) / 2, dst_y = (dst_h - h) / 2;
  const int src_x = (this->front_width_ - w) / 2, src_y = (this->front_height_ - h) / 2;
  const uint8_t *src = this->rgb_[this->front_];
  for (int row = 0; row < h; row++) {
    memcpy(buf->data + (dst_y + row) * dst_stride + dst_x * 2,
           src + ((src_y + row) * this->front_stride_ + src_x) * 2, w * 2);
  }
  lv_obj_invalidate(canvas);
}
#endif

}  // namespace esp_rtsp_stream
}  // namespace esphome
