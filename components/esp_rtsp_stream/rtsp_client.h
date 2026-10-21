#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace esp_rtsp_stream {

struct RtspUrl {
  std::string host;
  uint16_t port{554};
  std::string uri;  // rtsp://host:port/path, never containing credentials
  std::string username;
  std::string password;
};

/// Parses rtsp://[user[:pass]@]host[:port][/path?query]. Percent-escapes in the user-info are decoded.
bool parse_rtsp_url(const std::string &url, RtspUrl &out, const char **error);

struct RtspResponse {
  int status{0};
  std::string headers;  // status line and header lines, CRLF separated
  std::string body;
  /// Case-insensitive lookup of the first header called `name`, trimmed; empty if absent.
  std::string header(const char *name) const;
};

struct SdpTrack {
  int payload_type{-1};  // -1: no such track
  std::string encoding;  // from a=rtpmap, e.g. "JPEG", "H264", "PCMU"
  int clock_rate{0};
  int channels{1};
  std::string control;  // media-level a=control
};

struct SdpDescription {
  std::string session_control;  // session-level a=control
  SdpTrack video;               // first m=video section
  SdpTrack audio;               // first m=audio section
};

/// Picks the first video and first audio section of an SDP body. Returns false if there is no video.
bool parse_sdp(const std::string &sdp, SdpDescription &out);
/// Resolves an a=control value against the DESCRIBE base URL (RFC 2326 C.1.1).
std::string resolve_control_url(const std::string &base, const std::string &control);

/// Blocking RTSP control connection with RTP-over-TCP interleaving (RFC 2326 10.12) and Basic/Digest auth.
/// Used from a single task only.
class RtspConnection {
 public:
  void set_credentials(const std::string &username, const std::string &password);
  /// Idle time after which a blocked read gives up.
  void set_receive_timeout(uint32_t ms) { this->rx_timeout_ms_ = ms; }

  bool connect(const std::string &host, uint16_t port, uint32_t timeout_ms);
  void close();

  /// Sends a request and waits for its response, answering one 401 challenge if credentials are set.
  /// `extra_headers` must terminate every line with CRLF. Returns false only on transport errors.
  bool request(const char *method, const std::string &uri, const std::string &extra_headers, RtspResponse &resp);
  /// Sends a request without waiting; the stream loop skips the response when it arrives.
  bool send_only(const char *method, const std::string &uri, const std::string &extra_headers);
  void set_session(const std::string &session) { this->session_ = session; }

  bool read_byte(uint8_t &b);
  bool read_exact(uint8_t *dst, size_t len);
  bool skip(size_t len);
  /// Reads the rest of a response whose first byte was already consumed.
  bool read_response_tail(uint8_t first, RtspResponse &resp) { return this->read_response_(resp, first); }
  /// Takes the new challenge from a 401 received outside request(), e.g. for a keepalive.
  bool refresh_challenge(const RtspResponse &resp) { return this->parse_challenge_(resp); }

 protected:
  bool send_request_(const char *method, const std::string &uri, const std::string &extra_headers);
  bool read_response_(RtspResponse &resp, int first_byte);
  bool read_line_(std::string &line);
  int recv_some_(uint8_t *dst, size_t len);
  bool write_all_(const char *data, size_t len);
  bool parse_challenge_(const RtspResponse &resp);
  std::string authorization_(const char *method, const std::string &uri);

  int fd_{-1};
  uint8_t rbuf_[2048];
  size_t rpos_{0};
  size_t rlen_{0};
  uint32_t last_rx_ms_{0};
  uint32_t rx_timeout_ms_{10000};
  uint32_t cseq_{0};
  std::string session_;

  enum class AuthScheme : uint8_t { NONE, BASIC, DIGEST };
  std::string username_;
  std::string password_;
  AuthScheme auth_{AuthScheme::NONE};
  std::string realm_;
  std::string nonce_;
  std::string opaque_;
  bool qop_auth_{false};
  uint32_t nonce_count_{0};
};

}  // namespace esp_rtsp_stream
}  // namespace esphome
