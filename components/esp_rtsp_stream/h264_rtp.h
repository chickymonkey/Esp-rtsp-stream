#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace esp_rtsp_stream {

/// Rebuilds H.264 access units (Annex-B byte stream, 4-byte start codes) from RTP payloads (RFC 6184,
/// non-interleaved mode: single NAL units, STAP-A and FU-A) straight into a caller-owned buffer.
///
/// The caller reads each payload from the socket into the pointers this class hands out, so a payload is
/// copied only once. Parameter sets (SPS/PPS) are remembered, from the SDP or in-band, so they can be put in
/// front of an IDR picture that arrives without them, as the decoder needs them to start.
class H264Assembler {
 public:
  void set_buffer(uint8_t *buf, size_t capacity) {
    this->buf_ = buf;
    this->capacity_ = capacity;
    this->clear();
  }
  /// Forgets the parameter sets and takes them from an SDP a=fmtp line (sprop-parameter-sets), if present.
  void set_sdp_fmtp(const std::string &fmtp);

  /// Starts a new access unit with this RTP timestamp.
  void start(uint32_t timestamp);
  /// Drops the access unit in progress, if any.
  void clear();
  bool started() const { return this->started_; }
  uint32_t timestamp() const { return this->timestamp_; }

  /// Room for `n` more bytes at the end of the access unit, or nullptr (the unit is then marked corrupt).
  uint8_t *reserve(size_t n);
  /// Adds `n` bytes, written at the pointer returned by the last reserve(), to the access unit.
  void commit(size_t n) { this->len_ += n; }
  /// Records a complete NAL unit (header byte first, no start code) that was just written.
  void note_nal(const uint8_t *nal, size_t len);
  /// Records the type of a NAL unit whose first fragment was just written.
  void note_nal_type(uint8_t type);

  /// Where a STAP-A payload of `n` bytes can be read to before unpack_stap_a(), or nullptr if there is no room.
  uint8_t *stap_scratch(size_t n);
  /// Splits a STAP-A payload (without its NAL header byte), read to stap_scratch(), into the access unit.
  bool unpack_stap_a(const uint8_t *src, size_t len);

  bool in_fragment() const { return this->in_fu_; }
  void set_in_fragment(bool in_fu) { this->in_fu_ = in_fu; }
  void mark_corrupt() { this->corrupt_ = true; }

  bool corrupt() const { return this->corrupt_; }
  bool has_slice() const { return this->has_slice_; }
  bool has_idr() const { return this->has_idr_; }
  bool has_sps() const { return this->has_sps_; }
  const uint8_t *data() const { return this->buf_; }
  size_t length() const { return this->len_; }

  /// Known SPS and PPS as Annex-B, to put in front of an IDR access unit that lacks them; empty if unknown.
  const std::string &parameter_sets() const { return this->parameter_sets_; }
  /// profile_idc of the stream (66 Baseline, 77 Main, 100 High), or 0 if not known yet.
  int profile_idc() const { return this->profile_idc_; }

 protected:
  void set_parameter_set_(const uint8_t *nal, size_t len);

  uint8_t *buf_{nullptr};
  size_t capacity_{0};
  size_t len_{0};
  uint32_t timestamp_{0};
  bool started_{false};
  bool in_fu_{false};
  bool corrupt_{false};
  bool has_slice_{false};
  bool has_idr_{false};
  bool has_sps_{false};

  std::string sps_;
  std::string pps_;
  std::string parameter_sets_;
  int profile_idc_{0};
};

/// Name of an H.264 profile_idc, for log messages.
const char *h264_profile_name(int profile_idc);

}  // namespace esp_rtsp_stream
}  // namespace esphome
