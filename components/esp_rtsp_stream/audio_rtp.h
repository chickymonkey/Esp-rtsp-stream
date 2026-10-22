#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace esp_rtsp_stream {

/// Turns RTP audio payloads into 16-bit mono PCM at a fixed output rate.
///
/// Supported: G.711 mu-law (PCMU), G.711 A-law (PCMA) and linear PCM (L16, any rate, mono or stereo).
/// Stereo is mixed down to mono; the rate is converted with linear interpolation.
class RtpAudioDecoder {
 public:
  enum class Codec : uint8_t { NONE, PCMU, PCMA, L16 };

  /// Selects the codec from an SDP rtpmap encoding name. Returns false for unsupported codecs.
  bool configure(const std::string &encoding, uint32_t clock_rate, int channels, uint32_t output_rate);
  /// Forgets the resampler state, e.g. after a reconnect.
  void reset();

  /// Upper bound of the samples decode() writes for a payload of `payload_len` bytes.
  size_t max_output_samples(size_t payload_len) const;
  /// Decodes one RTP payload. Returns the number of samples written to `out`.
  size_t decode(const uint8_t *payload, size_t len, int16_t *out, size_t capacity);

  Codec codec() const { return this->codec_; }
  const char *codec_name() const;
  uint32_t input_rate() const { return this->input_rate_; }
  int input_channels() const { return this->channels_; }

 protected:
  size_t push_(int16_t sample, int16_t *out, size_t pos, size_t capacity);

  Codec codec_{Codec::NONE};
  uint32_t input_rate_{8000};
  uint32_t output_rate_{16000};
  int channels_{1};
  uint32_t step_{1u << 16};  // input samples per output sample, Q16
  uint32_t phase_{0};        // position between last_ and the next input sample, Q16
  int16_t last_{0};
};

}  // namespace esp_rtsp_stream
}  // namespace esphome
