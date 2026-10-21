# esp_rtsp_stream

ESPHome external component that shows an RTSP **MJPEG** camera stream (like Hikvision, Dahua and other ONVIF-style IP cameras) on an ESP32-S3 or ESP32-P4 display.

## Prerequisite
- **Dual Core** (currently ESP32-S3 S31 and P4 are equiped with this)
- **PSRAM**
- **MJPEG** stream configured on the camera, normally the sub-stream ( does not decode H.264/H.265 at this stage )

## How it works

1. **RTSP:** RTSP 1.0 runs over one TCP connection, with RTP interleaved on the same socket (`RTP/AVP/TCP`). This avoids lost UDP packets and firewall/NAT problems on WiFi. Basic and Digest authentication are supported (both cameras use Digest by default). An `OPTIONS` keepalive is sent every `timeout/2`.
2. **RTP/JPEG (RFC 2435):** JPEG frames are rebuilt directly in a PSRAM buffer. Supported: types 0/1 (4:2:2 and 4:2:0), types 64/65 (restart markers), Q 1–99 (standard tables) and Q 128–255 (in-band tables).
3. **Decoding:** a separate task decodes each frame to RGB565. A frame that arrives while the decoder is busy, or faster than `max_fps`, is dropped. The picture always shows the newest frame and never builds a backlog.
4. **Display:** decoded frames are double-buffered. `id(cam).draw(it)` draws the newest frame from a display lambda.

## Per-chip profiles

The profile is chosen automatically from the `esp32:` variant/board, so you don't need a "model" option.

| | ESP32-S3 | ESP32-P4 |
|---|---|---|
| Decoder | `espressif/esp_new_jpeg` (software, S3 SIMD) | Hardware JPEG engine |
| Scale / rotate | During decode (down to 1/8, multiples of 8) | PPA (1/16 steps) |
| Exact fit | A final bilinear resize closes the last few pixels | Same |
| Default `max_fps` | 12 | 25 |
| Default `max_frame_size` | 256 KB | 1 MB |
| Recommended stream | 640×480 or 704×576 (D1), MJPEG | Up to 1920×1080 MJPEG, link permitting |
| Expected rate | ~15–20 fps at D1/VGA, if WiFi keeps up | Limited by the network, not the decoder |
| lwIP TCP window | 32 KB | 64 KB |

The network task runs on core 0 and the decoder on core 1. PSRAM is required.

## Camera setup

| Camera | Setting | URL |
|---|---|---|
| Hikvision | Configuration → Video/Audio → *Sub Stream* → Video Encoding **MJPEG** | `rtsp://IP:554/Streaming/Channels/102` |
| Dahua | Setup → Camera → Video → *Sub Stream 1* → Encode Mode **MJPEG** | `rtsp://IP:554/cam/realmonitor?channel=1&subtype=1` |

RTP/JPEG cannot carry frames larger than 2040 px in either direction.

## Configuration

```yaml
external_components:
  - source: github://chickymonkey/Esp-rtsp-stream
    components: [esp_rtsp_stream]

esp_rtsp_stream:
  - id: front_door
    url: rtsp://192.168.1.64:554/Streaming/Channels/102
    username: admin                    # or put user:pass@ in the URL
    password: !secret camera_password
    output_width: 480                  # optional: fit (keeping aspect) into this box
    output_height: 320
    rotation: 0                        # 0/90/180/270, clockwise
    max_fps: 12                        # optional, default from chip profile
    max_frame_size: 262144             # optional, bytes per JPEG frame
    display_id: lcd                    # optional: redraw this display on every new frame
    log_frame_info: true               # fps/decode stats every 30 s
```

Use the `username`/`password` options when the password contains `@`, `/` or `:`.

### ESP32-S3 example

```yaml
esp32:
  variant: esp32s3
  cpu_frequency: 240MHZ
  framework:
    type: esp-idf

psram:
  mode: octal
  speed: 80MHz

display:
  - platform: ...            # your panel
    id: lcd
    update_interval: never   # esp_rtsp_stream triggers updates
    auto_clear_enabled: false
    lambda: |-
      id(front_door).draw(it);          // centred
      // id(front_door).draw(it, 0, 0); // or at a position
```

### ESP32-P4 example

```yaml
esp32:
  variant: esp32p4
  framework:
    type: esp-idf

psram:
  speed: 200MHz

# Network: ethernet (best for high resolution) or esp32_hosted + wifi
```

The display section is the same as for the S3, typically with a `mipi_dsi` panel. For a portrait 800×1280 panel showing a landscape camera, use `rotation: 90` and `output_width: 800` / `output_height: 1280`.



## LVGL

```yaml
esp_rtsp_stream:
  - id: front_door
    url: rtsp://192.168.1.64:554/Streaming/Channels/102
    lvgl_canvas: cam_canvas   # output size defaults to the canvas size

lvgl:
  pages:
    - id: cam_page
      widgets:
        - canvas:
            id: cam_canvas
            x: 0
            y: 0
            width: 320
            height: 240
```
## C++ API

- `draw(it)` / `draw(it, x, y)`: draw the newest frame.
- `has_frame()`, `is_streaming()`, `get_frame_width()`, `get_frame_height()`
- `get_frame_count()`, `get_dropped_frames()`, `get_last_decode_ms()`

## Limitations

- MJPEG only; no H.264/H.265.
- RTP over TCP only. A camera that refuses TCP transport fails at SETUP.
- 8-bit quantisation tables only; 16-bit tables are rare.
- With LVGL, use `lvgl_canvas` (below), not a display lambda: LVGL owns the screen.
The frame is copied into the canvas from the main loop, which is safe for LVGL, and centred on it. While the canvas isn't on screen (another page is showing), frames are not decoded. The RTSP session stays connected, so the picture comes back instantly.
