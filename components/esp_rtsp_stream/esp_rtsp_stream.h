#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_DISPLAY
#include "esphome/components/display/display.h"
#endif
#ifdef USE_LVGL
#include <lvgl.h>
#endif
#ifdef USE_ESP_RTSP_STREAM_AUDIO
#include <freertos/stream_buffer.h>
#include "esphome/components/speaker/speaker.h"
#endif

#include "rfc2435.h"
#include "rtsp_client.h"

#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#endif

namespace esphome {
namespace esp_rtsp_stream {

/// Pulls an MJPEG stream from an RTSP camera (RTP/JPEG over interleaved TCP) and decodes it to RGB565.
///
/// Two FreeRTOS tasks do the work: a network task that runs the RTSP session and rebuilds JPEG frames,
/// and a decode task. Frames arriving while the decoder is busy, or faster than max_fps, are dropped so
/// the picture never lags behind the camera. Decoded frames are double-buffered for draw().
class RtspMjpeg : public Component {
 public:
  void set_url(const std::string &url) { this->url_ = url; }
  void set_credentials(const std::string &username, const std::string &password) {
    this->username_ = username;
    this->password_ = password;
  }
  void set_max_fps(int fps) { this->max_fps_ = fps; }
  void set_output_size(int width, int height) {
    this->box_width_ = width;
    this->box_height_ = height;
  }
  void set_rotation(int degrees) { this->rotation_ = degrees; }
  void set_max_frame_size(size_t bytes) { this->max_frame_size_ = bytes; }
  void set_task_cores(int network_core, int decode_core) {
    this->network_core_ = network_core;
    this->decode_core_ = decode_core;
  }
  void set_log_frame_info(bool enabled) { this->log_frame_info_ = enabled; }
#ifdef USE_DISPLAY
  void set_display(display::Display *display) { this->display_ = display; }

  /// Draws the latest frame centred on the display. Call from the display lambda.
  void draw(display::Display &it);
  /// Draws the latest frame with its top-left corner at (x, y).
  void draw(display::Display &it, int x, int y);
#endif
#ifdef USE_LVGL
  /// Shows frames in an LVGL canvas. Takes the address of the widget pointer, which LVGL assigns later.
  void set_lvgl_canvas(lv_obj_t **canvas) { this->lvgl_canvas_ = canvas; }
#endif

#ifdef USE_ESP_RTSP_STREAM_AUDIO
  void set_speaker(speaker::Speaker *speaker) { this->speaker_ = speaker; }
  void set_audio_buffer_ms(uint32_t ms) { this->audio_buffer_ms_ = ms; }
#endif
  /// Turns camera audio on or off at runtime (e.g. from a switch or a button lambda).
  void set_audio_enabled(bool enabled) { this->audio_enabled_ = enabled; }
  bool is_audio_enabled() const { return this->audio_enabled_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool has_frame() const { return this->front_width_ > 0; }
  bool is_streaming() const { return this->streaming_; }
  int get_frame_width() const { return this->front_width_; }
  int get_frame_height() const { return this->front_height_; }
  uint32_t get_frame_count() const { return this->decoded_frames_; }
  uint32_t get_dropped_frames() const { return this->dropped_frames_; }
  uint32_t get_last_decode_ms() const { return this->last_decode_ms_; }

 protected:
  struct OutputPlan {
    float scale;          // PPA scale factor (P4)
    int target_width;     // exact aspect-preserving size, before rotation
    int target_height;
    int scaled_width;     // what the decoder / PPA produces, before rotation
    int scaled_height;
    int stage_width;      // scaled size after rotation
    int stage_height;
    int out_width;        // final frame size after rotation
    int out_height;
  };

  static void network_task_(void *arg);
  static void decode_task_(void *arg);
  void network_loop_();
  void decode_loop_();

  /// Runs one RTSP session until it fails. Returns true if at least one frame was received.
  bool run_session_();
  bool handle_rtp_(uint16_t len);
  void frame_complete_(size_t jpeg_len);

  OutputPlan plan_output_(int src_width, int src_height) const;
  bool decode_frame_(const uint8_t *jpeg, size_t len, int src_width, int src_height);
  bool ensure_buffer_(uint8_t *&buf, size_t &capacity, size_t bytes);
  /// Resizes scaled_ into rgb_[back] if the scaler missed the exact size, and records the frame size.
  bool finish_frame_(const OutputPlan &plan, int back);
#ifdef USE_LVGL
  void update_lvgl_canvas_(lv_obj_t *canvas);
#endif
#ifdef USE_ESP_RTSP_STREAM_AUDIO
  /// SETUPs the audio track if its codec is supported. Returns false only on a transport error.
  bool setup_audio_(const SdpTrack &track, const std::string &base);
  bool handle_audio_rtp_(uint16_t len);
  void pump_audio_();
  void drain_audio_();
#endif
  uint8_t *alloc_jpeg_buffer_(size_t bytes);
  void free_buffer_(uint8_t *buf);

  // Configuration
  std::string url_;
  std::string username_;
  std::string password_;
  int max_fps_{10};
  int box_width_{0};
  int box_height_{0};
  int rotation_{0};
  size_t max_frame_size_{256 * 1024};
  int network_core_{0};
  int decode_core_{1};
  bool log_frame_info_{true};
#ifdef USE_DISPLAY
  display::Display *display_{nullptr};
#endif
#ifdef USE_LVGL
  lv_obj_t **lvgl_canvas_{nullptr};
  int canvas_frame_width_{0};
  int canvas_frame_height_{0};
#endif
  // False while nothing shows the stream (e.g. the canvas page is hidden): frames are not decoded.
  std::atomic<bool> frames_wanted_{true};
  std::atomic<bool> audio_enabled_{true};

#ifdef USE_ESP_RTSP_STREAM_AUDIO
  enum class AudioCodec : uint8_t { NONE, PCMU, PCMA, L16 };
  static constexpr size_t AUDIO_RING_SIZE = 64 * 1024;  // 4 s at 8 kHz mono

  speaker::Speaker *speaker_{nullptr};
  uint32_t audio_buffer_ms_{200};

  // Set by the network task per session, read by the main loop.
  std::atomic<AudioCodec> audio_codec_{AudioCodec::NONE};
  std::atomic<uint32_t> audio_format_generation_{0};
  uint32_t audio_sample_rate_{8000};
  uint8_t audio_channels_{1};
  uint8_t audio_rtp_channel_{2};
  int audio_payload_type_{-1};
  std::atomic<bool> audio_wanted_{false};
  std::atomic<uint32_t> audio_overruns_{0};
  uint8_t audio_packet_[4096];

  StreamBufferHandle_t audio_ring_{nullptr};
  StaticStreamBuffer_t audio_ring_struct_;
  uint8_t *audio_ring_storage_{nullptr};

  // Main loop side
  uint8_t audio_out_[1024];
  size_t audio_out_len_{0};
  size_t audio_out_pos_{0};
  bool speaker_started_{false};
  uint32_t speaker_generation_{0};
#endif

  RtspUrl parsed_url_;
  RtspConnection conn_;
  MjpegAssembler assembler_;
  TaskHandle_t network_task_handle_{nullptr};
  TaskHandle_t decode_task_handle_{nullptr};

  // Network task state
  int video_payload_type_{26};
  uint8_t rtp_channel_{0};
  bool have_seq_{false};
  uint16_t last_seq_{0};
  uint8_t qtables_[256];
  uint32_t next_frame_due_ms_{0};
  uint32_t session_frames_{0};

  // JPEG hand-off: the network task fills jpeg_buffers_[fill_index_]; the decoder reads the other one.
  uint8_t *jpeg_buffers_[2]{nullptr, nullptr};
  int fill_index_{0};
  int ready_index_{0};
  size_t ready_len_{0};
  int ready_width_{0};
  int ready_height_{0};
  std::atomic<bool> decoder_busy_{false};

  // RGB565 double buffer: the decoder writes rgb_[front_ ^ 1], draw() reads rgb_[front_].
  std::mutex frame_mutex_;
  uint8_t *rgb_[2]{nullptr, nullptr};
  size_t rgb_capacity_[2]{0, 0};
  int back_width_{0};
  int back_height_{0};
  int back_stride_{0};
  int front_{0};
  int front_width_{0};
  int front_height_{0};
  int front_stride_{0};
  std::atomic<bool> new_frame_{false};
  // Scaler output awaiting the final exact-size resize.
  uint8_t *scaled_{nullptr};
  size_t scaled_capacity_{0};
  bool scale_fallback_logged_{false};

#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
  jpeg_decoder_handle_t hw_decoder_{nullptr};
  ppa_client_handle_t ppa_client_{nullptr};
  uint8_t *staging_{nullptr};
  size_t staging_capacity_{0};
#endif

  // Statistics
  std::atomic<bool> streaming_{false};
  std::atomic<uint32_t> received_frames_{0};
  std::atomic<uint32_t> decoded_frames_{0};
  std::atomic<uint32_t> dropped_frames_{0};
  std::atomic<uint32_t> corrupt_frames_{0};
  std::atomic<uint32_t> last_decode_ms_{0};
  int stream_width_{0};
  int stream_height_{0};
  uint32_t last_stats_ms_{0};
  uint32_t last_stats_decoded_{0};
  uint32_t last_warning_ms_{0};
};

}  // namespace esp_rtsp_stream
}  // namespace esphome
