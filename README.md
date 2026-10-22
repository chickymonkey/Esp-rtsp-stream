# esp_rtsp_stream

ESPHome external component that shows an RTSP **MJPEG** camera stream (Hikvision, Dahua and other ONVIF-style IP cameras) on an ESP32-S3 or ESP32-P4 display. It can also play the camera's **audio** track on an ESPHome speaker.

It does not decode H.264/H.265. Set the camera stream you use, normally the sub-stream, to **MJPEG**.

## How it works

1. **RTSP:** RTSP 1.0 runs over one TCP connection, with RTP interleaved on the same socket (`RTP/AVP/TCP`). This avoids lost UDP packets and firewall/NAT problems on WiFi. Basic and Digest authentication are supported (both cameras use Digest by default). An `OPTIONS` keepalive is sent every `timeout/2`.
2. **RTP/JPEG (RFC 2435):** JPEG frames are rebuilt directly in a PSRAM buffer. Supported: types 0/1 (4:2:2 and 4:2:0), types 64/65 (restart markers), Q 1–99 (standard tables) and Q 128–255 (in-band tables).
3. **Decoding:** a separate task decodes each frame to RGB565. A frame that arrives while the decoder is busy, or faster than `max_fps`, is dropped. The picture always shows the newest frame and never builds a backlog.
4. **Display:** decoded frames are double-buffered. `id(cam).draw(it)` draws the newest frame from a display lambda.
5. **Audio (optional):** if an `audio:` block is set, the camera's audio track is set up on the same TCP connection. G.711 or L16 audio is decoded, mixed to mono, resampled to the speaker's rate and sent to an ESPHome `speaker`. See [Audio](#audio).

## Per-chip profiles

The profile is chosen automatically from the `esp32:` variant/board, so you don't need a "model" option.

| | ESP32-S3 | ESP32-P4 |
|---|---|---|
| Decoder | `espressif/esp_new_jpeg` (software, S3 SIMD) | Hardware JPEG engine |
| Scale / rotate | Full-size decode, then 2×2 averaging + bilinear resize; rotation in the decoder | PPA (1/16 steps) plus a final bilinear resize |
| Default `max_fps` | 12 | 25 |
| Default `max_frame_size` | 256 KB | 1 MB |
| Recommended stream | 640×480 or 704×576 (D1), MJPEG | Up to 1920×1080 MJPEG, link permitting |
| Expected rate | ~15–20 fps at D1/VGA on an otherwise idle board, if WiFi keeps up. About 5–6 fps measured on a busy BOX-3; see [Tuning](#tuning). | Limited by the network, not the decoder |
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
| `hidden_timeout` | 3s | With `lvgl_canvas`: disconnect from the camera once the canvas has been hidden this long. `never` keeps the session open. |
| `display_id` | — | Non-LVGL display to redraw on every frame (draw with `id(cam).draw(it)`) |
| `output_width` / `output_height` | canvas size | Box the picture is fitted into, keeping its shape. Leave unset with `lvgl_canvas`. |
| `rotation` | 0 | 0/90/180/270, clockwise |
| `max_fps` | 12 (S3), 25 (P4) | Frames shown per second at most |
| `max_frame_size` | 256 KB (S3), 1 MB (P4) | Largest JPEG frame accepted |
| `log_frame_info` | true | Stats line every 30 s |
| `audio` | — | Play the camera audio on a speaker; see [Audio](#audio) |

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
- **Hidden canvas:** while another page is showing, frames are not decoded.
- **Disconnect when away:** once the canvas has been hidden for `hidden_timeout` (default 3 s), the ESP32 sends TEARDOWN and closes the RTSP session. That frees the camera's session slot, WiFi bandwidth and CPU. A quick page flick shorter than that keeps the session.
- **Coming back:** the stream reconnects as soon as the canvas shows again. The last picture stays on screen until the first new frame, usually within a second or two.
- **Always connected:** set `hidden_timeout: never` to keep the session open, so the picture returns instantly.

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

Then set the helper to `rtsp://FRIGATE_IP:8554/porch_esp`. The conversion runs on the server. To get audio as well, see [go2rtc / Frigate with audio](#go2rtc--frigate-with-audio).



## Audio

Audio is optional. Add an `audio:` block that points at an ESPHome `speaker`, and the camera's sound plays alongside the picture.

```yaml
esp_rtsp_stream:
  - id: front_door
    # ... as above ...
    audio:
      speaker: box_speaker     # any ESPHome speaker: i2s_audio, mixer, resampler ...
      sample_rate: 16000       # the rate your speaker runs at (default 16000)
      muted: false             # start muted (default false)
```

| Option | Default | Meaning |
|---|---|---|
| `speaker` | — | Speaker to play the camera audio on |
| `sample_rate` | 16000 | Rate the speaker runs at. Camera audio is resampled to it, so set it to the speaker's `sample_rate`. |
| `muted` | false | Start muted. Change at runtime with `set_audio_muted()`. |

How it behaves:
- **Same connection:** audio travels on the same RTSP TCP connection as the picture, on its own interleaved channel.
- **Follows the picture:** audio only plays while the picture is wanted. When the LVGL canvas page is hidden, the speaker is stopped.
- **No backlog:** packets that arrive while the speaker is full, muted or still starting are dropped. Sound never falls behind.
- **Optional track:** a camera without audio, or with an unsupported codec, logs one warning and streams video only.
- **Speaker buffer:** the speaker's `buffer_duration` absorbs WiFi jitter. The default of 500 ms is a good start.

### Supported audio codecs

| Codec | SDP name | Typical source |
|---|---|---|
| G.711 A-law | `PCMA` | Dahua "G.711A", Hikvision "G.711alaw", go2rtc `#audio=pcma` |
| G.711 µ-law | `PCMU` | Hikvision "G.711ulaw", go2rtc `#audio=pcmu` |
| Linear PCM | `L16` | go2rtc `#audio=pcm/16000`, any rate, mono or stereo |

AAC (the Dahua default), Opus and G.726 are not decoded. Change the camera's audio encoding, or let go2rtc convert it.

### Camera audio setup

| Camera | Setting |
|---|---|
| Dahua | Setup → Camera → Audio → *Sub Stream* → **Enable**, Encode Mode **G.711A** |
| Hikvision | Configuration → Video/Audio → *Sub Stream* → Stream Type **Video&Audio**; Audio → Audio Encoding **G.711alaw** |

### go2rtc / Frigate with audio

The Opus track that Frigate makes for browsers (`#audio=opus`) can't be played on the ESP32. Add a separate stream for the ESP32 with MJPEG video and G.711 audio:

```yaml
go2rtc:
  streams:
    PorchDahuaFrigate:
      - rtsp://viewer:PASSWORD@192.168.3.160:554/cam/realmonitor?channel=1&subtype=0
      - ffmpeg:PorchDahuaFrigate#audio=opus
    porch_esp:
      - "ffmpeg:PorchDahuaFrigate#video=mjpeg#audio=pcma"
```

Then set the helper to `rtsp://FRIGATE_IP:8554/porch_esp`. Use `#audio=pcm/16000` instead for slightly better sound with no resampling on a 16 kHz speaker. It uses about 256 kbit/s instead of 64 kbit/s.

### Example: ESP32-S3-BOX-3 speaker

The BOX-3 has an ES8311 codec and an amplifier switched by GPIO46. Check the pins against your board revision.

```yaml
i2c:
  - id: bus_a
    sda: GPIO8
    scl: GPIO18

i2s_audio:
  - id: i2s_audio_bus
    i2s_lrclk_pin: GPIO45
    i2s_bclk_pin: GPIO17
    i2s_mclk_pin: GPIO2

audio_dac:
  - platform: es8311
    id: es8311_dac
    bits_per_sample: 16bit
    sample_rate: 16000

speaker:
  - platform: i2s_audio
    id: box_speaker
    i2s_audio_id: i2s_audio_bus
    i2s_dout_pin: GPIO15
    dac_type: external
    sample_rate: 16000
    bits_per_sample: 16bit
    channel: left
    audio_dac: es8311_dac

switch:
  - platform: gpio
    name: Speaker amplifier
    pin: GPIO46
    restore_mode: ALWAYS_ON
```

If the speaker is also used by a voice assistant or media player, don't give `esp_rtsp_stream` the same speaker. Each would start, stop and write to it on its own. Put a `mixer` speaker in front of it and give each user its own source speaker:

```yaml
speaker:
  - platform: i2s_audio
    id: box_speaker
    # ... pins and audio_dac as above ...
    sample_rate: 48000
  - platform: mixer
    id: box_mixer                # not "mixer_speaker": that name clashes with ESPHome's C++ namespace
    output_speaker: box_speaker
    num_channels: 1
    source_speakers:
      - id: announcement_speaker   # media_player -> announcement_pipeline -> speaker
      - id: camera_speaker         # esp_rtsp_stream -> audio -> speaker

esp_rtsp_stream:
  - id: front_door
    audio:
      speaker: camera_speaker
      sample_rate: 48000           # the mixer runs at the output speaker's rate
```

### Mute switch

A template switch turns the camera audio on and off from Home Assistant. Make it `optimistic`, so its state and any icon on the screen change as soon as it is pressed. A switch that reads its state from a `lambda` is only polled every 60 s.

```yaml
esp_rtsp_stream:
  - id: front_door
    audio:
      speaker: camera_speaker
      sample_rate: 48000
      muted: true                  # start muted, matching the switch below

switch:
  - platform: template
    id: camera_audio_switch
    name: Camera RTSP audio
    icon: "mdi:cctv"
    optimistic: true
    restore_mode: ALWAYS_OFF
    turn_on_action:
      - lambda: id(front_door).set_audio_muted(false);
    turn_off_action:
      - lambda: id(front_door).set_audio_muted(true);
```

### Voice assistant and micro wake word (shared I2S bus)

On the ESP32-S3-BOX-3, the microphone and the speaker share one I2S bus, and ESPHome lets only one of them use it at a time. Micro wake word (mWW) keeps the microphone capturing all the time, so the camera audio can't start. The log then shows:

```
[E][i2s_audio.speaker.std:401]: Parent bus is busy
[E][i2s_audio.speaker:134]: Driver failed to start; retrying in 1 second
```

There is no way round this on that wiring: the shared clock pins can't run the microphone at 16 kHz and the speaker at 48 kHz at once. The media player has the same limit, which is why the BOX-3 voice assistant config stops the wake word in `on_announcement`. Camera audio needs the same treatment. The switch frees the microphone before it unmutes and gives it back after it mutes, so the wake word is off while camera audio plays.

The actions below use the `stop_wake_word` and `start_wake_word` scripts and the `wake_word_engine_location` select from the [ESPHome BOX-3 voice assistant config](https://github.com/esphome/wake-word-voice-assistants). Use the names from your own config if they differ.

```yaml
switch:
  - platform: template
    id: camera_audio_switch
    name: Camera RTSP audio
    icon: "mdi:cctv"
    optimistic: true
    restore_mode: ALWAYS_OFF          # matches muted: true in esp_rtsp_stream
    turn_on_action:
      # Free the I2S bus: stop the wake word, as on_announcement does
      - if:
          condition:
            - microphone.is_capturing:
          then:
            - script.execute: stop_wake_word
            - if:
                condition:
                  - lambda: return id(wake_word_engine_location).current_option() == "In Home Assistant";
                then:
                  - wait_until:
                      - not:
                          voice_assistant.is_running:
            - wait_until:
                condition:
                  - not:
                      microphone.is_capturing:
                timeout: 2s
      - lambda: id(front_door).set_audio_muted(false);
    turn_off_action:
      - lambda: id(front_door).set_audio_muted(true);
      # Wait for the speaker to release the bus, then listen again
      - wait_until:
          condition:
            - speaker.is_stopped: box_speaker
          timeout: 3s
      - script.execute: start_wake_word
    on_turn_on:
      - lvgl.label.update:
          id: cam_audio_icon
          text: "\U000F057E"   # mdi:volume-high
          text_color: 0x00FF00
    on_turn_off:
      - lvgl.label.update:
          id: cam_audio_icon
          text: "\U000F0581"   # mdi:volume-off
          text_color: white
```

Leave out `on_turn_on` and `on_turn_off` if you don't use the on-screen button below. Other things that restart the wake word, such as the media player's `on_idle` after an announcement, hit the same busy-bus error while camera audio is on. Turn camera audio off and on again if that happens.

### LVGL page with a mute button

This page shows the camera with a round mute button in its top-left corner. The button is listed after the canvas, so it is drawn on top of the picture. Leaving the page turns the audio off, which also hands the microphone back to the wake word.

```yaml
font:
  # Material Design Icons; any icon font with these two glyphs works
  - file: "https://github.com/Templarian/MaterialDesign-Webfont/raw/v7.4.47/fonts/materialdesignicons-webfont.ttf"
    id: icon_font_25
    size: 25
    glyphs:
      - "\U000F057E"   # mdi:volume-high
      - "\U000F0581"   # mdi:volume-off

lvgl:
  pages:
    - id: camera_page
      bg_color: black
      scrollbar_mode: "off"
      scrollable: false
      on_unload:
        # Only when audio is on; otherwise each page change would restart the wake word, even during music
        - if:
            condition:
              switch.is_on: camera_audio_switch
            then:
              - switch.turn_off: camera_audio_switch
      widgets:
        - canvas:
            id: cam_canvas
            x: 0
            y: 0
            width: 320
            height: 240

        - button:
            id: cam_audio_btn
            x: 4
            y: 4
            width: 40
            height: 40
            radius: 20
            bg_color: black
            bg_opa: 50%
            border_width: 0
            shadow_width: 0
            on_click:
              - switch.toggle: camera_audio_switch
            widgets:
              - label:
                  id: cam_audio_icon
                  align: CENTER
                  text_font: icon_font_25
                  text_color: white
                  text: "\U000F0581"   # starts muted
```

How it behaves:
- **Muted:** a white crossed-out speaker. The wake word listens as usual.
- **Tap to listen:** the icon turns into a green speaker. The wake word stops, then camera audio starts.
- **Tap again, or leave the page:** camera audio stops and the wake word starts again.
- **In sync:** the button and the Home Assistant switch always agree, because both drive the same switch.
- **Overlays:** anything on LVGL's `top_layer`, such as a voice assistant notice bar, is drawn above the button and can cover it.

## Reading the log

With `log_frame_info: true`, the component logs each step of a session and a stats line every 30 s. This log is from an ESP32-S3-BOX-3 showing a Dahua 640×480 MJPEG sub-stream with G.711A audio:

```
[I][esp_rtsp_stream]: Canvas shown: resuming the camera stream
[I][esp_rtsp_stream]: Audio track: G.711 A-law, 8000 Hz, 1 channel(s) -> speaker at 48000 Hz
[I][esp_rtsp_stream]: Streaming MJPEG with audio from 192.168.3.160
[I][esp_rtsp_stream]: First frame: 640x480 stream -> 300x225 output, decoded in 102 ms
[I][esp_rtsp_stream]: 640x480 stream -> 300x225, 5.6 fps shown, decode 86 ms, 214 dropped, 0 corrupt
[I][esp_rtsp_stream]: Audio G.711 A-law 8000 Hz -> 48000 Hz: muted, 1535 packets, 0 samples dropped
[I][esp_rtsp_stream]: Canvas hidden: pausing the camera stream
```

| Line | Meaning |
|---|---|
| `Canvas shown` / `Canvas hidden` | The stream resumed or paused because the LVGL page changed. See `hidden_timeout`. |
| `Audio track` | The camera's audio codec and rate, and the rate it is converted to. Missing if there is no `audio:` block or the camera has no audio. |
| `Streaming MJPEG` | PLAY succeeded. `with audio` means the audio track was set up too. |
| `First frame` | Camera size, output size and the time to decode the first frame. |
| `fps shown` | Frames drawn per second since the previous stats line. |
| `decode` | Time to decode and scale the latest frame. |
| `dropped` | Frames skipped since boot, because they arrived faster than `max_fps` or while the decoder was busy. Normal; it keeps the picture current. |
| `corrupt` | Frames lost to missing packets or bad JPEG data. Should stay at or near 0. |
| `muted` / `playing` / `paused` | Audio state: muted by `set_audio_muted()`, sent to the speaker, or stopped because the canvas is hidden. |
| `samples dropped` | Audio the speaker had no room for. Only counted while playing; a slowly rising number means the camera's clock runs slightly fast. |

Things that look wrong but aren't:
- **Low fps on the first stats line after a page change:** fps is averaged over the whole 30 s window, including the time the stream was paused.
- **A large `dropped` count:** it is a total since boot, and dropping is how the picture stays current.

### Tuning

- **Match the camera to `max_fps`:** every dropped frame still crossed the WiFi. Set the camera's sub-stream frame rate close to `max_fps` to save bandwidth with no visible change.
- **Decode time limits the frame rate:** the most frames per second is about 1000 divided by the `decode` time in ms. At 86 ms that is about 11 fps, and in practice less, because the network and the screen also need time.
- **Smaller pictures decode faster:** on the ESP32-S3 the decoder always decodes the full camera picture before scaling it. A 352×288 sub-stream decodes much faster than 640×480, and on a 320×240 screen it looks almost the same.
- **Busy boards are slower:** a voice assistant, wake word, media player and a busy LVGL UI all share the CPU and PSRAM. On the BOX-3 below, with all of them running, 640×480 decoded in about 86 ms.

## Limitations

- MJPEG only; no H.264/H.265.
- Audio must be G.711 (PCMA/PCMU) or L16. AAC, Opus and G.726 are not decoded; ( let go2rtc convert them if you have frigate installed).
- Audio and picture are not lip-synced. Each plays as soon as it arrives, which is close enough for a door camera in LAN.
- On boards where the microphone and speaker share one I2S bus, such as the ESP32-S3-BOX-3, the wake word is off while camera audio plays. See [Voice assistant and micro wake word](#voice-assistant-and-micro-wake-word-shared-i2s-bus).
- RTP over TCP only. A camera that refuses TCP transport fails at SETUP.
- 8-bit quantisation tables only.

## Tested Board-screen

- **ESP32-S3-BOX-3**, 2.4-inch 320×240 SPI TFT. Dahua 640×480 MJPEG sub-stream with G.711A audio, LVGL canvas 300×225, with voice assistant, micro wake word and media player running: about 5.6 fps, 86 ms decode, 0 corrupt frames.
  
## C++ API

- `set_url(url)`, `set_credentials(user, pass)`: switch camera at runtime.
- `draw(it)` / `draw(it, x, y)`: draw the newest frame (display lambda).
- `has_frame()`, `is_streaming()`, `is_paused()`, `get_frame_width()`, `get_frame_height()`
- `get_frame_count()`, `get_dropped_frames()`, `get_last_decode_ms()`
- `set_audio_muted(bool)`, `is_audio_muted()`, `is_audio_playing()`: camera audio control
