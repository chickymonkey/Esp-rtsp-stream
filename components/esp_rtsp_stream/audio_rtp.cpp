#include "audio_rtp.h"

#include <strings.h>

namespace esphome {
namespace esp_rtsp_stream {

static int16_t ulaw_to_linear(uint8_t u) {
  u = ~u;
  int t = ((u & 0x0F) << 3) + 0x84;
  t <<= (u & 0x70) >> 4;
  return static_cast<int16_t>((u & 0x80) ? 0x84 - t : t - 0x84);
}

static int16_t alaw_to_linear(uint8_t a) {
  a ^= 0x55;
  int t = (a & 0x0F) << 4;
  const int seg = (a & 0x70) >> 4;
  if (seg == 0) {
    t += 8;
  } else {
    t += 0x108;
    t <<= seg - 1;
  }
  return static_cast<int16_t>((a & 0x80) ? t : -t);
}

bool RtpAudioDecoder::configure(const std::string &encoding, uint32_t clock_rate, int channels,
                                uint32_t output_rate) {
  const char *name = encoding.c_str();
  if (strcasecmp(name, "PCMU") == 0) {
    this->codec_ = Codec::PCMU;
  } else if (strcasecmp(name, "PCMA") == 0) {
    this->codec_ = Codec::PCMA;
  } else if (strcasecmp(name, "L16") == 0) {
    this->codec_ = Codec::L16;
  } else {
    this->codec_ = Codec::NONE;
    return false;
  }
  this->input_rate_ = clock_rate > 0 ? clock_rate : 8000;
  // G.711 is always mono; L16 may be interleaved stereo.
  this->channels_ = this->codec_ == Codec::L16 && channels > 1 ? channels : 1;
  this->output_rate_ = output_rate > 0 ? output_rate : this->input_rate_;
  this->step_ = static_cast<uint32_t>((static_cast<uint64_t>(this->input_rate_) << 16) / this->output_rate_);
  if (this->step_ == 0)
    this->step_ = 1;
  this->reset();
  return true;
}

void RtpAudioDecoder::reset() {
  this->phase_ = 0;
  this->last_ = 0;
}

const char *RtpAudioDecoder::codec_name() const {
  switch (this->codec_) {
    case Codec::PCMU:
      return "G.711 mu-law";
    case Codec::PCMA:
      return "G.711 A-law";
    case Codec::L16:
      return "L16 PCM";
    default:
      return "none";
  }
}

size_t RtpAudioDecoder::max_output_samples(size_t payload_len) const {
  const size_t bytes_per_frame = this->codec_ == Codec::L16 ? 2 * this->channels_ : 1;
  const uint64_t frames = payload_len / bytes_per_frame;
  return static_cast<size_t>((frames << 16) / this->step_) + 2;
}

size_t RtpAudioDecoder::push_(int16_t sample, int16_t *out, size_t pos, size_t capacity) {
  // Emit every output sample that falls between last_ and this input sample.
  const int32_t delta = static_cast<int32_t>(sample) - this->last_;
  while (this->phase_ < (1u << 16)) {
    if (pos < capacity)
      out[pos++] = static_cast<int16_t>(this->last_ + ((static_cast<int64_t>(delta) * this->phase_) >> 16));
    this->phase_ += this->step_;
  }
  this->phase_ -= 1u << 16;
  this->last_ = sample;
  return pos;
}

size_t RtpAudioDecoder::decode(const uint8_t *payload, size_t len, int16_t *out, size_t capacity) {
  size_t pos = 0;
  switch (this->codec_) {
    case Codec::PCMU:
      for (size_t i = 0; i < len; i++)
        pos = this->push_(ulaw_to_linear(payload[i]), out, pos, capacity);
      break;
    case Codec::PCMA:
      for (size_t i = 0; i < len; i++)
        pos = this->push_(alaw_to_linear(payload[i]), out, pos, capacity);
      break;
    case Codec::L16: {
      // Network byte order, channels interleaved.
      const size_t frame_bytes = 2 * this->channels_;
      for (size_t i = 0; i + frame_bytes <= len; i += frame_bytes) {
        int32_t sum = 0;
        for (int c = 0; c < this->channels_; c++)
          sum += static_cast<int16_t>((payload[i + 2 * c] << 8) | payload[i + 2 * c + 1]);
        pos = this->push_(static_cast<int16_t>(sum / this->channels_), out, pos, capacity);
      }
      break;
    }
    default:
      break;
  }
  return pos;
}

}  // namespace esp_rtsp_stream
}  // namespace esphome
