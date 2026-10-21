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

  SdpVideoTrack track;
  if (!parse_sdp_video(resp.body, track)) {
    ESP_LOGE(TAG, "Camera offers no video track");
    return false;
  }
  if (strcasecmp(track.encoding.c_str(), "JPEG") != 0) {
    ESP_LOGE(TAG, "Stream is %s, not MJPEG. Set this camera stream's encoding to MJPEG (usually the sub-stream)",
             track.encoding.empty() ? "an unknown codec" : track.encoding.c_str());
    return false;
  }
  this->video_payload_type_ = track.payload_type;

  std::string base = resp.header("Content-Base");
  if (base.empty())
    base = resp.header("Content-Location");
  if (base.empty())
    base = url.uri;
  const std::string setup_uri = resolve_control_url(base, track.control);
  const std::string play_uri = resolve_control_url(base, track.session_control);

  if (!this->conn_.request("SETUP", setup_uri, "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n", resp))
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
  const std::string transport = resp.header("Transport");
  const size_t interleaved = transport.find("interleaved=");
  this->rtp_channel_ = interleaved == std::string::npos ? 0 : atoi(transport.c_str() + interleaved + 12);
  this->conn_.set_session(session);

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
      const bool ok = hdr[0] == this->rtp_channel_ ? this->handle_rtp_(len) : this->conn_.skip(len);
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
  // esp_new_jpeg always decodes at full size: its scale option produced frames whose layout did not match
  // the size it reported (left/right halves stacked). finish_frame_() halves and resizes instead.
  plan.scale = scale;
  plan.scaled_width = src_width;
  plan.scaled_height = src_height;
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
    const OutputPlan plan = this->plan_output_(src_width, src_height);
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
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
    // The frame is read back as stage_width x stage_height pixels; anything else would scramble the picture.
    if (err == JPEG_ERR_OK && out_len != plan.stage_width * plan.stage_height * 2) {
      if (!this->scale_fallback_logged_) {
        this->scale_fallback_logged_ = true;
        ESP_LOGW(TAG, "Decoder output is %d bytes, expected %dx%d; dropping frames", out_len, plan.stage_width,
                 plan.stage_height);
      }
      err = JPEG_ERR_FAIL;
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
