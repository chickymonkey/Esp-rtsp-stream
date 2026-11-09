#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
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
#include "esphome/components/speaker/speaker.h"
#endif
#include "audio_rtp.h"
#include "h264_rtp.h"
#include "rfc2435.h"
#include "rtsp_client.h"

#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#endif
#ifdef USE_ESP_RTSP_STREAM_H264
#include "esp_h264_dec_sw.h"
#endif

namespace esphome {
namespace esp_rtsp_stream {

/// Pulls an MJPEG or H.264 stream from an RTSP camera (RTP over interleaved TCP) and decodes it to RGB565.
///
/// Two FreeRTOS tasks do the work: a network task that runs the RTSP session and rebuilds JPEG frames or
/// H.264 access units, and a decode task. MJPEG frames arriving while the decoder is busy, or faster than
/// max_fps, are dropped so the picture never lags behind the camera. H.264 pictures depend on each other, so
/// every access unit is queued and decoded; only the conversion to RGB565 is limited to max_fps, and if the
/// queue overflows the stream skips ahead to the next IDR picture. Decoded frames are double-buffered for draw().
class RtspMjpeg : public Component {
 public:
  /// Sets the camera URL; safe to call at runtime (e.g. from a Home Assistant text sensor) and reconnects to
  /// the new camera. Invalid values such as "unavailable" are ignored, so the current stream keeps running.
  void set_url(const std::string &url);
  /// Default login, used when the URL itself carries no user:pass@. Also safe to call at runtime.
  void set_credentials(const std::string &username, const std::string &password);
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
  /// With lvgl_canvas: disconnect from the camera once the canvas has been hidden this long. UINT32_MAX = never.
  void set_hidden_timeout(uint32_t ms) { this->hidden_timeout_ms_ = ms; }
#ifdef USE_ESP_RTSP_STREAM_AUDIO
  /// Plays the camera's audio track on this speaker.
  void set_speaker(speaker::Speaker *speaker) { this->speaker_ = speaker; }
  /// Rate the speaker runs at; camera audio is converted to it.
  void set_audio_sample_rate(uint32_t rate) { this->audio_sample_rate_ = rate; }
#endif
  /// Mutes or unmutes the camera audio. Safe to call at runtime; the RTSP session is not interrupted.
  void set_audio_muted(bool muted) { this->audio_muted_ = muted; }
  bool is_audio_muted() const { return this->audio_muted_; }
  /// True while camera audio is being sent to the speaker.
  bool is_audio_playing() const { return this->audio_wanted_; }
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

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool has_frame() const { return this->front_width_ > 0; }
  bool is_streaming() const { return this->streaming_; }
  /// True while the stream is disconnected because the canvas is hidden.
  bool is_paused() const { return !this->connection_wanted_; }
  int get_frame_width() const { return this->front_width_; }
  int get_frame_height() const { return this->front_height_; }
  uint32_t get_frame_count() const { return this->decoded_frames_; }
  uint32_t get_dropped_frames() const { return this->dropped_frames_; }
  uint32_t get_last_decode_ms() const { return this->last_decode_ms_; }

 protected:
  enum class VideoCodec : uint8_t { MJPEG, H264 };

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
  /// Takes the latest URL/credentials into the network task. Returns false if there is no usable URL yet.
  bool apply_config_();
  /// Tells the network task that the URL or credentials changed.
  void request_reconnect_();
  void decode_loop_();

  /// Runs one RTSP session until it fails. Returns true if at least one frame was received.
  bool run_session_();
  bool handle_rtp_(uint16_t len);
  /// Reads one interleaved audio packet and plays it.
  bool handle_audio_rtp_(uint16_t len);
  /// Starts or stops the speaker to match the session and mute state. Main loop only.
  void update_audio_();
  void frame_complete_(size_t jpeg_len);
  /// True if a frame should be shown now, at most max_fps; advances `next_due_ms` when it is.
  bool frame_due_(uint32_t &next_due_ms);
#ifdef USE_ESP_RTSP_STREAM_H264
  /// Reads one H.264 RTP payload (the `len` bytes after the RTP header) into the access unit being built.
  bool handle_h264_payload_(size_t len, bool marker, uint32_t timestamp, bool padding, bool lost);
  /// Queues the finished access unit for the decode task.
  void h264_access_unit_complete_();
  bool ensure_h264_queue_();
  void log_h264_profile_();
#endif

  OutputPlan plan_output_(int src_width, int src_height) const;
  /// Decodes the JPEG frame handed over by the network task and shows it.
  void decode_ready_jpeg_();
  /// Makes the frame just written to the back buffer the shown one. `lock` may already hold frame_mutex_.
  void publish_frame_(std::unique_lock<std::mutex> &lock, int src_width, int src_height);
  bool decode_frame_(const uint8_t *jpeg, size_t len, int src_width, int src_height);
#ifdef USE_ESP_RTSP_STREAM_HW_JPEG
  /// PPA scale/rotate of the RGB565 picture in staging_ (pic_w x pic_h, of which block_w x block_h is used).
  bool scale_staging_with_ppa_(const OutputPlan &plan, int back, int pic_w, int pic_h, int block_w, int block_h);
#endif
#ifdef USE_ESP_RTSP_STREAM_H264
  /// Decodes every queued H.264 access unit.
  void drain_h264_queue_();
  void decode_h264_access_unit_(const uint8_t *item, size_t size);
  /// Converts a decoded I420 picture into the back frame buffer, scaled and rotated.
  bool present_i420_(const uint8_t *buf, size_t size, int width, int height);
  bool open_h264_decoder_();
  void close_h264_decoder_();
#endif
  bool ensure_buffer_(uint8_t *&buf, size_t &capacity, size_t bytes);
  /// Resizes scaled_ into rgb_[back] if the scaler missed the exact size, and records the frame size.
  bool finish_frame_(const OutputPlan &plan, int back);
#ifdef USE_LVGL
  void update_lvgl_canvas_(lv_obj_t *canvas);
#endif
  uint8_t *alloc_jpeg_buffer_(size_t bytes);
  void free_buffer_(uint8_t *buf);
  /// Frees the decode/frame buffers while the stream is paused. Decode task only.
  void release_frame_buffers_();

  // Configuration
  // Written by set_url()/set_credentials() on the main loop, read by the network task.
  std::mutex config_mutex_;
  std::string url_;
  std::string username_;
  std::string password_;
  std::atomic<bool> config_changed_{false};
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
  // False once the canvas has been hidden for hidden_timeout_ms_: the network task disconnects.
  std::atomic<bool> connection_wanted_{true};
  uint32_t hidden_timeout_ms_{3000};
  uint32_t last_visible_ms_{0};

  RtspUrl parsed_url_;
  RtspConnection conn_;
  MjpegAssembler assembler_;
  TaskHandle_t network_task_handle_{nullptr};
  TaskHandle_t decode_task_handle_{nullptr};

  // Network task state
  VideoCodec codec_{VideoCodec::MJPEG};
  int video_payload_type_{26};
  uint8_t rtp_channel_{0};
  bool have_seq_{false};
  uint16_t last_seq_{0};
  uint8_t qtables_[256];
  uint32_t next_frame_due_ms_{0};
  uint32_t session_frames_{0};

  // H.264. The network task builds access units in jpeg_buffers_[fill_index_] and queues them in a ring buffer
  // (an H264QueueHeader, then the Annex-B data); the decode task decodes all of them in order.
  H264Assembler h264_assembler_;
  std::atomic<RingbufHandle_t> h264_queue_{nullptr};
  // Bumped for every H.264 session; queued units of an older session are skipped and the decoder restarts.
  std::atomic<uint32_t> h264_generation_{0};
#ifdef USE_ESP_RTSP_STREAM_H264
  uint8_t *h264_queue_storage_{nullptr};
  StaticRingbuffer_t h264_queue_struct_;
  size_t h264_max_item_{0};
  bool h264_need_idr_{true};  // network task: skip access units until the next IDR picture
  bool h264_profile_logged_{false};
  bool h264_unsupported_nal_logged_{false};
  // Decode task
  esp_h264_dec_handle_t h264_decoder_{nullptr};
  esp_h264_dec_param_sw_handle_t h264_param_{nullptr};
  uint32_t h264_decoder_generation_{0};
  bool h264_synced_{false};  // an IDR picture was decoded since the decoder (re)started
  uint32_t h264_next_present_ms_{0};
  bool h264_layout_logged_{false};
  bool h264_open_error_logged_{false};
#endif

  // Audio. The network task decodes packets and hands them to the speaker; the main loop starts and stops it.
  RtpAudioDecoder audio_decoder_;
  uint8_t audio_channel_{255};  // interleaved channel of the audio track, 255 when there is none
  int audio_payload_type_{-1};
  uint8_t *audio_packet_{nullptr};
  int16_t *audio_pcm_{nullptr};
  size_t audio_pcm_capacity_{0};
  std::atomic<bool> audio_track_active_{false};  // this session has an audio track
  std::atomic<bool> audio_wanted_{false};        // the speaker is started for us
  std::atomic<bool> audio_muted_{false};
  std::atomic<uint32_t> audio_packets_{0};
  std::atomic<uint32_t> audio_dropped_samples_{0};
  uint32_t audio_sample_rate_{16000};
  bool audio_unsupported_logged_{false};
#ifdef USE_ESP_RTSP_STREAM_AUDIO
  speaker::Speaker *speaker_{nullptr};
  bool speaker_started_{false};
  uint32_t last_audio_start_ms_{0};
#endif

  // JPEG hand-off: the network task fills jpeg_buffers_[fill_index_]; the decoder reads the other one.
  uint8_t *jpeg_buffers_[2]{nullptr, nullptr};
  int fill_index_{0};
  int ready_index_{0};
  size_t ready_len_{0};
  int ready_width_{0};
  int ready_height_{0};
  std::atomic<bool> decoder_busy_{false};

  // RGB565 double buffer: the decoder writes rgb_[front_ ^ 1], draw() reads rgb_[front_].
  // With only an LVGL canvas the canvas holds the shown copy, so a single buffer (rgb_[front_]) is used
  // and the decoder holds frame_mutex_ while it writes.
  std::mutex frame_mutex_;
  uint8_t *rgb_[2]{nullptr, nullptr};
  size_t rgb_capacity_[2]{0, 0};
  bool single_buffer_{false};
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
  uint32_t last_alloc_error_ms_{0};
};

}  // namespace esp_rtsp_stream
}  // namespace esphome
