# esp_rtsp_stream

ESPHome external component that shows an RTSP **MJPEG** camera stream (Hikvision, Dahua and other ONVIF-style IP cameras) on an ESP32-S3 or ESP32-P4 display.

It does not decode H.264/H.265. Set the camera stream you use, normally the sub-stream, to **MJPEG**.

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
| Scale / rotate | Full-size decode, then 2×2 averaging + bilinear resize; rotation in the decoder | PPA (1/16 steps) plus a final bilinear resize |
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

**Picture:** a 640×480 sub-stream at 7–10 fps works well on a 320×240 screen; it is halved exactly to fill it.

## Configuration

The camera URL lives in a Home Assistant text helper, so switching cameras needs no reflash.

### 1. Home Assistant: text helper

Go to Settings → Devices & services → Helpers → **Create helper → Text**, or use `configuration.yaml`:

```yaml
input_text:
  esp_camera_url:
    name: ESP camera URL
    max: 255
    pattern: 'rtsp:\/\/([^:@\/\s]+(:[^@\/\s]*)?@)?((25[0-5]|2[0-4]\d|1?\d?\d)\.){3}(25[0-5]|2[0-4]\d|1?\d?\d):\d{1,5}(\/\S*)?'
```

Set its value to the camera URL, for example `rtsp://192.168.3.160:554/cam/realmonitor?channel=1&subtype=1`.

- **`max: 255`:** the default of 100 is too short for some URLs.
- **`pattern`:** the editor only accepts `rtsp://`, an IPv4 address and a port, with an optional `user:pass@` and path. When entering it in the UI (⚙️ → *Regex pattern for client-side validation*), paste it without the quotes. Keep the escaped `\/`: browsers check this field in a regex mode where an unescaped `/` inside `[...]` makes the pattern invalid, and the check then silently stops working. To allow hostnames such as `doorcam.local`, replace the IP part with `[A-Za-z0-9.\-]+`.

### 2. ESPHome

```yaml
external_components:
  - source: github://chickymonkey/Esp-rtsp-stream
    components: [esp_rtsp_stream]

esp_rtsp_stream:
  - id: front_door
    username: viewer                   # default login when the URL has none
    password: !secret camera_password
    lvgl_canvas: cam_canvas            # see "Showing the picture"
    max_fps: 7                         # optional, default from chip profile
    max_frame_size: 262144             # optional, bytes per JPEG frame
    log_frame_info: true               # fps/decode stats every 30 s

text_sensor:
  - platform: homeassistant
    id: cam_url
    entity_id: input_text.esp_camera_url
    on_value:
      - lambda: id(front_door).set_url(x);
```

All options:

| Option | Default | Meaning |
|---|---|---|
| `url` | — | Optional fixed or fallback URL, streamed at boot until Home Assistant sends one |
| `username` / `password` | — | Login used when the URL has no `user:pass@` |
| `lvgl_canvas` | — | LVGL canvas to show the picture in |
| `display_id` | — | Non-LVGL display to redraw on every frame (draw with `id(cam).draw(it)`) |
| `output_width` / `output_height` | canvas size | Box the picture is fitted into, keeping its shape. Leave unset with `lvgl_canvas`. |
| `rotation` | 0 | 0/90/180/270, clockwise |
| `max_fps` | 12 (S3), 25 (P4) | Frames shown per second at most |
| `max_frame_size` | 256 KB (S3), 1 MB (P4) | Largest JPEG frame accepted |
| `log_frame_info` | true | Stats line every 30 s |

### Switching cameras

Change the helper's value in Home Assistant and the ESP32 reconnects to the new camera straight away.

- **Valid new URL:** the current session is dropped and the new camera connects.
- **Same URL re-sent** (e.g. after a Home Assistant reconnect): ignored, so the picture isn't interrupted.
- **Invalid value** (e.g. `unavailable` while Home Assistant restarts): ignored, and the current camera keeps streaming.
- **No URL yet:** the component waits.
- **Until the first frame arrives:** the previous camera's last picture stays on screen.

**Login:** credentials in the URL (`rtsp://user:pass@host:554/...`) take precedence over `username`/`password`. One helper can therefore switch between cameras with different logins, but that password is then visible in Home Assistant. If your cameras share a login, keep credentials out of the URL.

## Showing the picture

### LVGL

```yaml
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

How the picture fits the canvas:
- **Fit:** the picture is scaled to fit the canvas, keeping its shape, and centred. It is never stretched or cropped.
- **Black bars:** shown where the shapes differ.
- **No zoom-in:** the picture is never enlarged beyond its native size.

How it updates:
- **Copying:** frames are copied into the canvas from the main loop, which is safe for LVGL.
- **Hidden canvas:** while another page is showing, frames are not decoded. The RTSP session stays connected, so the picture returns instantly.

### Display lambda (without LVGL)

```yaml
display:
  - platform: ...            # your panel
    id: lcd
    update_interval: never   # esp_rtsp_stream triggers updates
    auto_clear_enabled: false
    lambda: |-
      id(front_door).draw(it);          // centred
      // id(front_door).draw(it, 0, 0); // or at a position
```

Add `display_id: lcd` to `esp_rtsp_stream`. With LVGL, use `lvgl_canvas` instead: LVGL owns the screen.

## Board setup

### ESP32-S3

```yaml
esp32:
  variant: esp32s3
  cpu_frequency: 240MHZ
  framework:
    type: esp-idf

psram:
  mode: octal
  speed: 80MHz
```

### ESP32-P4

```yaml
esp32:
  variant: esp32p4
  framework:
    type: esp-idf

psram:
  speed: 200MHz

# Network: ethernet (best for high resolution) or esp32_hosted + wifi
```

For a portrait 800×1280 panel showing a landscape camera, use `rotation: 90`.

## Cameras without MJPEG (go2rtc / Frigate)

The ESP32 can't decode H.264. go2rtc can convert a stream to MJPEG for it:

```yaml
go2rtc:
  streams:
    porch_esp:
      - "ffmpeg:PorchDahuaFrigate#video=mjpeg"
```

Then set the helper to `rtsp://FRIGATE_IP:8554/porch_esp`. The conversion runs on the server.



## Limitations

- MJPEG only; no H.264/H.265, and no audio.
- RTP over TCP only. A camera that refuses TCP transport fails at SETUP.
- 8-bit quantisation tables only; 16-bit tables are rare.

## Tested Board-screen

- ESP32-S3 Box3 - 2.4-inch color TFT SPI 320x240
  
## C++ API

- `set_url(url)`, `set_credentials(user, pass)`: switch camera at runtime.
- `draw(it)` / `draw(it, x, y)`: draw the newest frame (display lambda).
- `has_frame()`, `is_streaming()`, `get_frame_width()`, `get_frame_height()`
- `get_frame_count()`, `get_dropped_frames()`, `get_last_decode_ms()`
