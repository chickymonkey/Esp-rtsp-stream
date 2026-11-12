#include "h264_rtp.h"

#include <cstdlib>
#include <cstring>

namespace esphome {
namespace esp_rtsp_stream {

static const uint8_t START_CODE[4] = {0, 0, 0, 1};

static int base64_value(char c) {
  if (c >= 'A' && c <= 'Z')
    return c - 'A';
  if (c >= 'a' && c <= 'z')
    return c - 'a' + 26;
  if (c >= '0' && c <= '9')
    return c - '0' + 52;
  if (c == '+' || c == '-')
    return 62;
  if (c == '/' || c == '_')
    return 63;
  return -1;
}

static std::string base64_decode(const std::string &in) {
  std::string out;
  uint32_t acc = 0;
  int bits = 0;
  for (char c : in) {
    const int v = base64_value(c);
    if (v < 0)
      continue;  // '=' padding, whitespace
    acc = (acc << 6) | v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out += static_cast<char>((acc >> bits) & 0xFF);
    }
  }
  return out;
}

const char *h264_profile_name(int profile_idc) {
  switch (profile_idc) {
    case 66:
      return "Baseline";
    case 77:
      return "Main";
    case 88:
      return "Extended";
    case 100:
      return "High";
    case 110:
      return "High 10";
    case 122:
      return "High 4:2:2";
    case 244:
      return "High 4:4:4";
    default:
      return "unknown";
  }
}

void H264Assembler::set_sdp_fmtp(const std::string &fmtp) {
  this->sps_.clear();
  this->pps_.clear();
  this->parameter_sets_.clear();
  this->profile_idc_ = 0;

  // sprop-parameter-sets=<base64 SPS>,<base64 PPS>[,...]
  const char *const key = "sprop-parameter-sets=";
  size_t pos = fmtp.find(key);
  if (pos != std::string::npos) {
    pos += strlen(key);
    const size_t end = fmtp.find(';', pos);
    const std::string value = fmtp.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
    size_t start = 0;
    while (start <= value.size()) {
      size_t comma = value.find(',', start);
      if (comma == std::string::npos)
        comma = value.size();
      const std::string nal = base64_decode(value.substr(start, comma - start));
      if (!nal.empty())
        this->set_parameter_set_(reinterpret_cast<const uint8_t *>(nal.data()), nal.size());
      start = comma + 1;
    }
  }

  // profile-level-id=<profile_idc><constraints><level_idc> in hex, when there is no SPS to read it from.
  pos = fmtp.find("profile-level-id=");
  if (this->profile_idc_ == 0 && pos != std::string::npos) {
    const std::string hex = fmtp.substr(pos + 17, 2);
    this->profile_idc_ = static_cast<int>(strtol(hex.c_str(), nullptr, 16));
  }
}

void H264Assembler::set_parameter_set_(const uint8_t *nal, size_t len) {
  const uint8_t type = nal[0] & 0x1F;
  std::string *target = type == 7 ? &this->sps_ : type == 8 ? &this->pps_ : nullptr;
  if (target == nullptr)
    return;
  if (target->size() == len && memcmp(target->data(), nal, len) == 0)
    return;  // unchanged; cameras repeat them before every IDR
  target->assign(reinterpret_cast<const char *>(nal), len);
  if (type == 7 && len > 1)
    this->profile_idc_ = nal[1];

  this->parameter_sets_.clear();
  if (!this->sps_.empty() && !this->pps_.empty()) {
    const std::string sc(reinterpret_cast<const char *>(START_CODE), sizeof(START_CODE));
    this->parameter_sets_ = sc + this->sps_ + sc + this->pps_;
  }
}

void H264Assembler::start(uint32_t timestamp) {
  this->clear();
  this->started_ = true;
  this->timestamp_ = timestamp;
}

void H264Assembler::clear() {
  this->len_ = 0;
  this->started_ = false;
  this->in_fu_ = false;
  this->corrupt_ = false;
  this->has_slice_ = false;
  this->has_idr_ = false;
  this->has_sps_ = false;
}

uint8_t *H264Assembler::reserve(size_t n) {
  if (this->corrupt_ || this->buf_ == nullptr || this->len_ + n > this->capacity_) {
    this->corrupt_ = true;
    return nullptr;
  }
  return this->buf_ + this->len_;
}

void H264Assembler::note_nal_type(uint8_t type) {
  switch (type) {
    case 5:  // IDR slice
      this->has_idr_ = true;
      this->has_slice_ = true;
      break;
    case 1:  // non-IDR slice
    case 2:  // slice data partition A
      this->has_slice_ = true;
      break;
    case 7:
      this->has_sps_ = true;
      break;
    default:
      break;
  }
}

void H264Assembler::note_nal(const uint8_t *nal, size_t len) {
  if (len == 0)
    return;
  const uint8_t type = nal[0] & 0x1F;
  this->note_nal_type(type);
  if (type == 7 || type == 8)
    this->set_parameter_set_(nal, len);
}

uint8_t *H264Assembler::stap_scratch(size_t n) {
  // Read the payload to the end of the buffer, then unpack it forwards: each NAL grows by 2 bytes (2-byte size
  // prefix -> 4-byte start code), so with this much room the writes never catch up with the unread data.
  if (this->corrupt_ || this->buf_ == nullptr || this->len_ + 3 * n + 8 > this->capacity_) {
    this->corrupt_ = true;
    return nullptr;
  }
  return this->buf_ + this->capacity_ - n;
}

bool H264Assembler::unpack_stap_a(const uint8_t *src, size_t len) {
  size_t pos = 0;
  while (pos + 2 <= len) {
    const size_t nal_len = (src[pos] << 8) | src[pos + 1];
    pos += 2;
    if (nal_len == 0 || pos + nal_len > len) {
      this->corrupt_ = true;
      return false;
    }
    uint8_t *dst = this->buf_ + this->len_;
    memmove(dst + sizeof(START_CODE), src + pos, nal_len);  // may overlap the scratch area
    memcpy(dst, START_CODE, sizeof(START_CODE));
    this->len_ += sizeof(START_CODE) + nal_len;
    this->note_nal(dst + sizeof(START_CODE), nal_len);
    pos += nal_len;
  }
  return true;
}

}  // namespace esp_rtsp_stream
}  // namespace esphome
