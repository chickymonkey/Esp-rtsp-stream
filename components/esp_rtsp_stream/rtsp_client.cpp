#include "rtsp_client.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>

#include "esp_rom_md5.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp_rtsp_stream {

static const char *const TAG = "esp_rtsp_stream.rtsp";

static bool starts_with_ci(const std::string &s, const char *prefix, size_t pos = 0) {
  for (size_t i = 0; prefix[i] != '\0'; i++) {
    if (pos + i >= s.size() || std::tolower((unsigned char) s[pos + i]) != std::tolower((unsigned char) prefix[i]))
      return false;
  }
  return true;
}

static std::string trim(const std::string &s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace((unsigned char) s[b]))
    b++;
  while (e > b && std::isspace((unsigned char) s[e - 1]))
    e--;
  return s.substr(b, e - b);
}

static std::string percent_decode(const std::string &s) {
  std::string out;
  for (size_t i = 0; i < s.size(); i++) {
    if (s[i] == '%' && i + 2 < s.size() && std::isxdigit((unsigned char) s[i + 1]) &&
        std::isxdigit((unsigned char) s[i + 2])) {
      out += static_cast<char>(strtol(s.substr(i + 1, 2).c_str(), nullptr, 16));
      i += 2;
    } else {
      out += s[i];
    }
  }
  return out;
}

static std::string md5_hex(const std::string &s) {
  md5_context_t ctx;
  uint8_t digest[ESP_ROM_MD5_DIGEST_LEN];
  esp_rom_md5_init(&ctx);
  esp_rom_md5_update(&ctx, s.data(), s.size());
  esp_rom_md5_final(digest, &ctx);
  char hex[2 * ESP_ROM_MD5_DIGEST_LEN + 1];
  for (int i = 0; i < ESP_ROM_MD5_DIGEST_LEN; i++)
    snprintf(hex + 2 * i, 3, "%02x", digest[i]);
  return hex;
}

static std::string base64(const std::string &in) {
  static const char *const CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < in.size(); i += 3) {
    uint32_t v = ((uint8_t) in[i] << 16) | ((uint8_t) in[i + 1] << 8) | (uint8_t) in[i + 2];
    out += CHARS[(v >> 18) & 63];
    out += CHARS[(v >> 12) & 63];
    out += CHARS[(v >> 6) & 63];
    out += CHARS[v & 63];
  }
  if (i < in.size()) {
    uint32_t v = (uint8_t) in[i] << 16;
    if (i + 1 < in.size())
      v |= (uint8_t) in[i + 1] << 8;
    out += CHARS[(v >> 18) & 63];
    out += CHARS[(v >> 12) & 63];
    out += i + 1 < in.size() ? CHARS[(v >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

/// Value of `key` in a comma separated `key=value` / `key="value"` list (as in WWW-Authenticate).
static std::string auth_param(const std::string &params, const char *key) {
  const size_t key_len = strlen(key);
  size_t pos = 0;
  while (pos < params.size()) {
    while (pos < params.size() && (params[pos] == ',' || std::isspace((unsigned char) params[pos])))
      pos++;
    size_t eq = params.find('=', pos);
    if (eq == std::string::npos)
      return "";
    const bool match = trim(params.substr(pos, eq - pos)).size() == key_len && starts_with_ci(params, key, pos);
    size_t vstart = eq + 1;
    std::string value;
    size_t next;
    if (vstart < params.size() && params[vstart] == '"') {
      size_t close = params.find('"', vstart + 1);
      if (close == std::string::npos)
        close = params.size();
      value = params.substr(vstart + 1, close - vstart - 1);
      next = params.find(',', close);
    } else {
      next = params.find(',', vstart);
      value = trim(params.substr(vstart, next == std::string::npos ? std::string::npos : next - vstart));
    }
    if (match)
      return value;
    if (next == std::string::npos)
      break;
    pos = next + 1;
  }
  return "";
}

bool parse_rtsp_url(const std::string &url, RtspUrl &out, const char **error) {
  if (!starts_with_ci(url, "rtsp://")) {
    *error = "URL must start with rtsp://";
    return false;
  }
  std::string rest = url.substr(7);
  const size_t slash = rest.find('/');
  std::string authority = slash == std::string::npos ? rest : rest.substr(0, slash);
  const std::string path = slash == std::string::npos ? "/" : rest.substr(slash);

  // The password may itself contain '@' if not percent-encoded, so split on the last one.
  const size_t at = authority.rfind('@');
  if (at != std::string::npos) {
    const std::string userinfo = authority.substr(0, at);
    authority = authority.substr(at + 1);
    const size_t colon = userinfo.find(':');
    out.username = percent_decode(userinfo.substr(0, colon));
    out.password = colon == std::string::npos ? "" : percent_decode(userinfo.substr(colon + 1));
  }

  const size_t colon = authority.rfind(':');
  out.port = 554;
  if (colon != std::string::npos && authority.find(']') == std::string::npos) {
    const std::string port = authority.substr(colon + 1);
    char *end = nullptr;
    long value = strtol(port.c_str(), &end, 10);
    if (port.empty() || *end != '\0' || value < 1 || value > 65535) {
      *error = "invalid RTSP port";
      return false;
    }
    out.port = static_cast<uint16_t>(value);
    authority = authority.substr(0, colon);
  }
  if (authority.empty()) {
    *error = "missing RTSP host";
    return false;
  }
  out.host = authority;
  out.uri = "rtsp://" + out.host + ":" + std::to_string(out.port) + path;
  return true;
}

std::string RtspResponse::header(const char *name) const {
  const size_t name_len = strlen(name);
  size_t pos = 0;
  while (pos < this->headers.size()) {
    size_t eol = this->headers.find("\r\n", pos);
    if (eol == std::string::npos)
      eol = this->headers.size();
    if (starts_with_ci(this->headers, name, pos) && pos + name_len < eol && this->headers[pos + name_len] == ':')
      return trim(this->headers.substr(pos + name_len + 1, eol - pos - name_len - 1));
    pos = eol + 2;
  }
  return "";
}

bool parse_sdp(const std::string &sdp, SdpDescription &out) {
  SdpTrack *current = nullptr;  // null: session level, or a section that is not used
  bool in_media = false;
  size_t pos = 0;
  while (pos < sdp.size()) {
    size_t eol = sdp.find('\n', pos);
    if (eol == std::string::npos)
      eol = sdp.size();
    const std::string line = trim(sdp.substr(pos, eol - pos));
    pos = eol + 1;

    if (line.rfind("m=", 0) == 0) {
      in_media = true;
      // Only the first section of each kind is used.
      if (line.rfind("m=video ", 0) == 0 && out.video.payload_type < 0) {
        current = &out.video;
      } else if (line.rfind("m=audio ", 0) == 0 && out.audio.payload_type < 0) {
        current = &out.audio;
      } else {
        current = nullptr;
        continue;
      }
      // m=<media> <port> <proto> <fmt> ...
      size_t fmt = line.find(' ', 2);
      fmt = fmt == std::string::npos ? fmt : line.find(' ', fmt + 1);
      fmt = fmt == std::string::npos ? fmt : line.find(' ', fmt + 1);
      current->payload_type = fmt == std::string::npos ? -1 : atoi(line.c_str() + fmt + 1);
      continue;
    }
    if (line.rfind("a=control:", 0) == 0) {
      if (!in_media)
        out.session_control = trim(line.substr(10));
      else if (current != nullptr)
        current->control = trim(line.substr(10));
    } else if (current != nullptr && line.rfind("a=rtpmap:", 0) == 0) {
      // a=rtpmap:<pt> <encoding>/<clock rate>[/<channels>]
      const size_t space = line.find(' ');
      if (atoi(line.c_str() + 9) != current->payload_type || space == std::string::npos)
        continue;
      const size_t slash = line.find('/', space);
      current->encoding = line.substr(space + 1, slash == std::string::npos ? std::string::npos : slash - space - 1);
      if (slash != std::string::npos) {
        current->clock_rate = atoi(line.c_str() + slash + 1);
        const size_t slash2 = line.find('/', slash + 1);
        if (slash2 != std::string::npos)
          current->channels = std::max(1, atoi(line.c_str() + slash2 + 1));
      }
    }
  }

  // Static payload types (RFC 3551) need no rtpmap.
  if (out.video.encoding.empty() && out.video.payload_type == 26) {
    out.video.encoding = "JPEG";
    out.video.clock_rate = 90000;
  }
  if (out.audio.encoding.empty() && (out.audio.payload_type == 0 || out.audio.payload_type == 8)) {
    out.audio.encoding = out.audio.payload_type == 0 ? "PCMU" : "PCMA";
    out.audio.clock_rate = 8000;
  }
  return out.video.payload_type >= 0;
}

std::string resolve_control_url(const std::string &base, const std::string &control) {
  if (control.empty() || control == "*")
    return base;
  if (starts_with_ci(control, "rtsp://"))
    return control;
  if (control[0] == '/') {
    const size_t path = base.find('/', 7);
    return (path == std::string::npos ? base : base.substr(0, path)) + control;
  }
  return base.back() == '/' ? base + control : base + "/" + control;
}

void RtspConnection::set_credentials(const std::string &username, const std::string &password) {
  this->username_ = username;
  this->password_ = password;
}

bool RtspConnection::connect(const std::string &host, uint16_t port, uint32_t timeout_ms) {
  this->close();

  struct addrinfo hints {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo *res = nullptr;
  const std::string port_str = std::to_string(port);
  if (lwip_getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || res == nullptr) {
    ESP_LOGW(TAG, "Cannot resolve %s", host.c_str());
    return false;
  }

  int fd = lwip_socket(res->ai_family, res->ai_socktype, 0);
  if (fd < 0) {
    lwip_freeaddrinfo(res);
    ESP_LOGW(TAG, "socket() failed: errno %d", errno);
    return false;
  }

  lwip_fcntl(fd, F_SETFL, lwip_fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  int rc = lwip_connect(fd, res->ai_addr, res->ai_addrlen);
  lwip_freeaddrinfo(res);
  if (rc != 0 && errno == EINPROGRESS) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    rc = lwip_select(fd + 1, nullptr, &wfds, nullptr, &tv) == 1 ? 0 : -1;
    if (rc == 0) {
      int err = 0;
      socklen_t len = sizeof(err);
      lwip_getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
      rc = err == 0 ? 0 : -1;
    }
  }
  if (rc != 0) {
    lwip_close(fd);
    ESP_LOGW(TAG, "Cannot connect to %s:%u", host.c_str(), port);
    return false;
  }
  lwip_fcntl(fd, F_SETFL, lwip_fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);

  // Short receive timeout so blocked reads can enforce rx_timeout_ms_ themselves.
  struct timeval rcv_tv;
  rcv_tv.tv_sec = 1;
  rcv_tv.tv_usec = 0;
  lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));
  struct timeval snd_tv;
  snd_tv.tv_sec = 5;
  snd_tv.tv_usec = 0;
  lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));
  int one = 1;
  lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  this->fd_ = fd;
  this->rpos_ = this->rlen_ = 0;
  this->cseq_ = 0;
  this->session_.clear();
  this->auth_ = AuthScheme::NONE;
  this->nonce_count_ = 0;
  this->last_rx_ms_ = millis();
  return true;
}

void RtspConnection::close() {
  if (this->fd_ >= 0) {
    lwip_close(this->fd_);
    this->fd_ = -1;
  }
}

bool RtspConnection::write_all_(const char *data, size_t len) {
  while (len > 0) {
    int n = lwip_send(this->fd_, data, len, 0);
    if (n <= 0) {
      ESP_LOGW(TAG, "send failed: errno %d", errno);
      return false;
    }
    data += n;
    len -= n;
  }
  return true;
}

int RtspConnection::recv_some_(uint8_t *dst, size_t len) {
  while (this->fd_ >= 0) {
    int n = lwip_recv(this->fd_, dst, len, 0);
    if (n > 0) {
      this->last_rx_ms_ = millis();
      return n;
    }
    if (n == 0) {
      ESP_LOGW(TAG, "Camera closed the connection");
      return -1;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      ESP_LOGW(TAG, "recv failed: errno %d", errno);
      return -1;
    }
    if (millis() - this->last_rx_ms_ > this->rx_timeout_ms_) {
      ESP_LOGW(TAG, "No data from camera for %u ms", (unsigned) this->rx_timeout_ms_);
      return -1;
    }
  }
  return -1;
}

bool RtspConnection::read_byte(uint8_t &b) {
  if (this->rpos_ == this->rlen_) {
    int n = this->recv_some_(this->rbuf_, sizeof(this->rbuf_));
    if (n < 0)
      return false;
    this->rpos_ = 0;
    this->rlen_ = n;
  }
  b = this->rbuf_[this->rpos_++];
  return true;
}

bool RtspConnection::read_exact(uint8_t *dst, size_t len) {
  while (len > 0) {
    if (this->rpos_ < this->rlen_) {
      size_t n = std::min(len, this->rlen_ - this->rpos_);
      memcpy(dst, this->rbuf_ + this->rpos_, n);
      this->rpos_ += n;
      dst += n;
      len -= n;
      continue;
    }
    // Large reads go straight into the destination (the JPEG frame buffer) to avoid a copy.
    if (len >= 512) {
      int n = this->recv_some_(dst, len);
      if (n < 0)
        return false;
      dst += n;
      len -= n;
      continue;
    }
    int n = this->recv_some_(this->rbuf_, sizeof(this->rbuf_));
    if (n < 0)
      return false;
    this->rpos_ = 0;
    this->rlen_ = n;
  }
  return true;
}

bool RtspConnection::skip(size_t len) {
  while (len > 0) {
    if (this->rpos_ < this->rlen_) {
      size_t n = std::min(len, this->rlen_ - this->rpos_);
      this->rpos_ += n;
      len -= n;
      continue;
    }
    int n = this->recv_some_(this->rbuf_, sizeof(this->rbuf_));
    if (n < 0)
      return false;
    this->rpos_ = 0;
    this->rlen_ = n;
  }
  return true;
}

bool RtspConnection::read_line_(std::string &line) {
  line.clear();
  uint8_t c;
  while (this->read_byte(c)) {
    if (c == '\n') {
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      return true;
    }
    line += static_cast<char>(c);
    if (line.size() > 4096) {
      ESP_LOGW(TAG, "RTSP header line too long");
      return false;
    }
  }
  return false;
}

bool RtspConnection::read_response_(RtspResponse &resp, int first_byte) {
  resp = RtspResponse{};
  std::string line;
  if (!this->read_line_(line))
    return false;
  if (first_byte >= 0)
    line.insert(line.begin(), static_cast<char>(first_byte));
  if (line.rfind("RTSP/", 0) != 0) {
    ESP_LOGW(TAG, "Unexpected response: %s", line.c_str());
    return false;
  }
  const size_t sp = line.find(' ');
  resp.status = sp == std::string::npos ? 0 : atoi(line.c_str() + sp + 1);
  resp.headers = line + "\r\n";

  while (true) {
    if (!this->read_line_(line))
      return false;
    if (line.empty())
      break;
    resp.headers += line + "\r\n";
  }

  const std::string content_length = resp.header("Content-Length");
  const long body_len = content_length.empty() ? 0 : atol(content_length.c_str());
  if (body_len < 0 || body_len > 32768) {
    ESP_LOGW(TAG, "Unreasonable Content-Length %ld", body_len);
    return false;
  }
  if (body_len > 0) {
    resp.body.resize(body_len);
    if (!this->read_exact(reinterpret_cast<uint8_t *>(&resp.body[0]), body_len))
      return false;
  }
  return true;
}

bool RtspConnection::send_request_(const char *method, const std::string &uri, const std::string &extra_headers) {
  std::string req;
  req.reserve(512);
  req += method;
  req += ' ';
  req += uri;
  req += " RTSP/1.0\r\nCSeq: ";
  req += std::to_string(++this->cseq_);
  req += "\r\nUser-Agent: ESPHome-esp_rtsp_stream\r\n";
  if (!this->session_.empty())
    req += "Session: " + this->session_ + "\r\n";
  if (this->auth_ != AuthScheme::NONE)
    req += this->authorization_(method, uri);
  req += extra_headers;
  req += "\r\n";
  ESP_LOGV(TAG, "> %s %s", method, uri.c_str());
  return this->write_all_(req.data(), req.size());
}

bool RtspConnection::send_only(const char *method, const std::string &uri, const std::string &extra_headers) {
  return this->send_request_(method, uri, extra_headers);
}

bool RtspConnection::request(const char *method, const std::string &uri, const std::string &extra_headers,
                             RtspResponse &resp) {
  for (int attempt = 0; attempt < 2; attempt++) {
    if (!this->send_request_(method, uri, extra_headers))
      return false;
    // Interleaved RTP may already be flowing (e.g. a late response); skip it until the reply arrives.
    while (true) {
      uint8_t first;
      if (!this->read_byte(first))
        return false;
      if (first != '$') {
        if (!this->read_response_(resp, first))
          return false;
        break;
      }
      uint8_t hdr[3];
      if (!this->read_exact(hdr, 3) || !this->skip((hdr[1] << 8) | hdr[2]))
        return false;
    }
    ESP_LOGV(TAG, "< %d for %s", resp.status, method);
    if (resp.status != 401 || this->username_.empty() || attempt == 1)
      return true;
    if (!this->parse_challenge_(resp))
      return true;
  }
  return true;
}

bool RtspConnection::parse_challenge_(const RtspResponse &resp) {
  bool have_basic = false;
  size_t pos = 0;
  const std::string &h = resp.headers;
  while (pos < h.size()) {
    size_t eol = h.find("\r\n", pos);
    if (eol == std::string::npos)
      eol = h.size();
    if (starts_with_ci(h, "WWW-Authenticate:", pos)) {
      const std::string value = trim(h.substr(pos + 17, eol - pos - 17));
      if (starts_with_ci(value, "Digest")) {
        const std::string params = value.substr(6);
        this->auth_ = AuthScheme::DIGEST;
        this->realm_ = auth_param(params, "realm");
        this->nonce_ = auth_param(params, "nonce");
        this->opaque_ = auth_param(params, "opaque");
        const std::string qop = auth_param(params, "qop");
        this->qop_auth_ = false;
        for (size_t p = 0; (p = qop.find("auth", p)) != std::string::npos; p += 4) {
          if (p + 4 == qop.size() || qop[p + 4] == ',' || qop[p + 4] == ' ')
            this->qop_auth_ = true;
        }
        this->nonce_count_ = 0;
        return true;  // Digest wins over Basic
      }
      if (starts_with_ci(value, "Basic")) {
        have_basic = true;
        this->realm_ = auth_param(value.substr(5), "realm");
      }
    }
    pos = eol + 2;
  }
  if (have_basic) {
    this->auth_ = AuthScheme::BASIC;
    return true;
  }
  ESP_LOGW(TAG, "Camera asked for an unsupported authentication scheme");
  return false;
}

std::string RtspConnection::authorization_(const char *method, const std::string &uri) {
  if (this->auth_ == AuthScheme::BASIC)
    return "Authorization: Basic " + base64(this->username_ + ":" + this->password_) + "\r\n";

  const std::string ha1 = md5_hex(this->username_ + ":" + this->realm_ + ":" + this->password_);
  const std::string ha2 = md5_hex(std::string(method) + ":" + uri);
  std::string header = "Authorization: Digest username=\"" + this->username_ + "\", realm=\"" + this->realm_ +
                       "\", nonce=\"" + this->nonce_ + "\", uri=\"" + uri + "\"";
  if (this->qop_auth_) {
    char nc[9], cnonce[9];
    snprintf(nc, sizeof(nc), "%08x", (unsigned) ++this->nonce_count_);
    snprintf(cnonce, sizeof(cnonce), "%08x", (unsigned) random_uint32());
    header += ", response=\"" + md5_hex(ha1 + ":" + this->nonce_ + ":" + nc + ":" + cnonce + ":auth:" + ha2) +
              "\", qop=auth, nc=" + nc + ", cnonce=\"" + cnonce + "\"";
  } else {
    header += ", response=\"" + md5_hex(ha1 + ":" + this->nonce_ + ":" + ha2) + "\"";
  }
  if (!this->opaque_.empty())
    header += ", opaque=\"" + this->opaque_ + "\"";
  return header + "\r\n";
}

}  // namespace esp_rtsp_stream
}  // namespace esphome
