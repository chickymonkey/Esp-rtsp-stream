import os
import shutil

import esphome.codegen as cg
from esphome.components import display, esp32, speaker
from esphome.components.esp32.const import VARIANT_ESP32P4, VARIANT_ESP32S3
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISPLAY_ID,
    CONF_ID,
    CONF_PASSWORD,
    CONF_ROTATION,
    CONF_URL,
    CONF_USERNAME,
)
from esphome.core import CORE
from esphome.coroutine import coroutine_with_priority
import esphome.final_validate as fv

try:
    from esphome.components.lvgl.widgets.canvas import lv_canvas_t
except ImportError:  # ESPHome without LVGL 9 canvas support
    lv_canvas_t = None

DEPENDENCIES = ["esp32", "network"]
MULTI_CONF = True

esp_rtsp_stream_ns = cg.esphome_ns.namespace("esp_rtsp_stream")
RtspMjpeg = esp_rtsp_stream_ns.class_("RtspMjpeg", cg.Component)

CONF_MAX_FPS = "max_fps"
CONF_OUTPUT_WIDTH = "output_width"
CONF_OUTPUT_HEIGHT = "output_height"
CONF_MAX_FRAME_SIZE = "max_frame_size"
CONF_LOG_FRAME_INFO = "log_frame_info"
CONF_LVGL_CANVAS = "lvgl_canvas"
CONF_HIDDEN_TIMEOUT = "hidden_timeout"
CONF_AUDIO = "audio"
CONF_SPEAKER = "speaker"
CONF_SAMPLE_RATE = "sample_rate"
CONF_MUTED = "muted"
CONF_H264 = "h264"

# Software H.264 decoder (tinyh264 based, Baseline profile only). Neither chip has an H.264 decoder in
# hardware; the P4 only has an encoder. Published speeds at 640x480: ~9 fps on ESP32-S3, ~25 fps on ESP32-P4.
# A range, not a pin: espressif/esp_video (used by P4 camera/intercom packages) requires esp_h264 1.3.*, so the
# solver must be free to pick 1.3.x there. The decoder API is identical from 1.3.0 to 1.5.x.
H264_IDF_COMPONENT = ("espressif/esp_h264", "^1.3.8")

AUDIO_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SPEAKER): cv.use_id(speaker.Speaker),
        # The rate the speaker runs at. Camera audio (usually 8 kHz G.711) is resampled to it.
        cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(min=8000, max=48000),
        cv.Optional(CONF_MUTED, default=False): cv.boolean,
    }
)


def _lvgl_canvas_id(value):
    if lv_canvas_t is None:
        raise cv.Invalid("lvgl_canvas needs an ESPHome version with LVGL 9 canvas support")
    return cv.use_id(lv_canvas_t)(value)

# Per-chip tuning, picked from the `esp32:` variant/board.
#  - ESP32-S3: software decode (esp_new_jpeg, SIMD). ~20 fps at 640x480, ~15 fps at 800x480,
#    so stay at D1/VGA sub-stream resolution and let the decoder shrink to the panel.
#  - ESP32-P4: hardware JPEG decoder (1080p class) plus PPA for scale/rotate, so larger
#    frames and higher frame rates are fine; the network link becomes the limit.
MODEL_PROFILES = {
    VARIANT_ESP32S3: {
        "max_fps": 12,
        "max_frame_size": 256 * 1024,
        "network_core": 0,  # next to the WiFi stack
        "decode_core": 1,
        "tcp_window": 32768,
        "tcp_recvmbox": 32,
        "idf_component": ("espressif/esp_new_jpeg", "1.0.2"),
        "define": None,
    },
    VARIANT_ESP32P4: {
        "max_fps": 25,
        "max_frame_size": 1024 * 1024,
        "network_core": 0,
        "decode_core": 1,
        "tcp_window": 65534,
        "tcp_recvmbox": 64,
        "idf_component": None,
        "define": "USE_ESP_RTSP_STREAM_HW_JPEG",
    },
}


def _profile():
    return MODEL_PROFILES[esp32.get_esp32_variant()]


def _validate_variant(config):
    variant = esp32.get_esp32_variant()
    if variant not in MODEL_PROFILES:
        raise cv.Invalid(
            f"esp_rtsp_stream supports ESP32-S3 and ESP32-P4, not {variant}"
        )
    return config


def _validate_url(value):
    value = cv.string_strict(value)
    if not value.lower().startswith("rtsp://"):
        raise cv.Invalid("URL must start with rtsp://")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RtspMjpeg),
            # Optional: the URL can also be set at runtime with set_url(), e.g. from Home Assistant.
            cv.Optional(CONF_URL): _validate_url,
            cv.Inclusive(CONF_USERNAME, "credentials"): cv.string_strict,
            cv.Inclusive(CONF_PASSWORD, "credentials"): cv.string_strict,
            cv.Optional(CONF_MAX_FPS): cv.int_range(min=1, max=30),
            cv.Inclusive(CONF_OUTPUT_WIDTH, "output_size"): cv.int_range(
                min=8, max=2048
            ),
            cv.Inclusive(CONF_OUTPUT_HEIGHT, "output_size"): cv.int_range(
                min=8, max=2048
            ),
            cv.Optional(CONF_ROTATION, default=0): cv.one_of(0, 90, 180, 270, int=True),
            cv.Optional(CONF_MAX_FRAME_SIZE): cv.int_range(
                min=16 * 1024, max=4 * 1024 * 1024
            ),
            cv.Optional(CONF_DISPLAY_ID): cv.use_id(display.Display),
            cv.Optional(CONF_LVGL_CANVAS): _lvgl_canvas_id,
            # Disconnect once the canvas has been hidden this long; "never" keeps the session open.
            cv.Optional(CONF_HIDDEN_TIMEOUT, default="3s"): cv.Any(
                cv.one_of("never", lower=True), cv.positive_time_period_milliseconds
            ),
            cv.Optional(CONF_LOG_FRAME_INFO, default=True): cv.boolean,
            cv.Optional(CONF_AUDIO): AUDIO_SCHEMA,
            # Also accept H.264 (Baseline profile) streams. Off by default: it adds the esp_h264 decoder.
            cv.Optional(CONF_H264, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_variant,
)


def _final_validate(config):
    if "psram" not in fv.full_config.get():
        raise cv.Invalid(
            "esp_rtsp_stream needs PSRAM for its frame buffers; add a `psram:` block"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


def _sdkconfig_default(name, value):
    # Keep any value the user set in esp32 -> framework -> sdkconfig_options.
    setter = getattr(esp32, "set_idf_sdkconfig_default", None)
    if setter is not None:
        setter(name, value)
    else:
        esp32.add_idf_sdkconfig_option(name, value)


# Public include dirs of the registry components we use, relative to the component's root.
IDF_COMPONENT_INCLUDE_DIRS = {
    "espressif/esp_h264": ["interface/include", "sw/include", "port/include", "hw/include"],
    "espressif/esp_new_jpeg": ["include"],
}

# Functions we call from a registry component. src cannot declare the dependency on 2026.10.x, so the link
# order of its archive is arbitrary: when another component (esp_video on the P4) also requires it, it lands
# before libsrc.a and the linker never goes back for these. -Wl,-u makes them required from the start.
IDF_COMPONENT_LINK_SYMBOLS = {
    "espressif/esp_h264": [
        "esp_h264_dec_sw_new",
        "esp_h264_dec_sw_get_param_hd",
        "esp_h264_dec_open",
        "esp_h264_dec_process",
        "esp_h264_dec_get_resolution",
        "esp_h264_dec_close",
        "esp_h264_dec_del",
    ],
}


def _use_idf_component(name, ref):
    """Adds a registry IDF component and puts its headers on the include path of ESPHome's code.

    Native ESP-IDF builds (ESPHome 2026.10+) only download components added with add_idf_component: the
    generated `src` component REQUIRES built-in IDF components only, so a managed component's public include
    dirs never reach our sources. ESP-IDF still builds and links the component, so only the headers are
    missing.
    """
    esp32.add_idf_component(name=name, ref=ref)
    done = CORE.data.setdefault("esp_rtsp_stream_linked", set())
    if name in done:
        return  # MULTI_CONF: once per component, not per stream
    done.add(name)
    idf_name = name.replace("/", "__")  # espressif/esp_h264 -> espressif__esp_h264

    add_src_cmake = getattr(esp32, "add_src_cmake", None)
    if add_src_cmake is not None:
        # Newer ESPHome: link the component into src, which also brings its public include dirs.
        var = "esp_rtsp_stream_" + idf_name.replace("-", "_")
        add_src_cmake(
            f"idf_component_get_property({var} {idf_name} COMPONENT_LIB)\n"
            f"target_link_libraries(${{COMPONENT_LIB}} PRIVATE ${{{var}}})\n"
        )
        return

    # ESPHome 2026.10.x has no hook into src's CMake, and only passes -D/-W build flags to the compiler.
    # -Wp,-I<dir> is GCC's way of handing -I<dir> to the preprocessor, so it gets through. The component
    # manager downloads into <build>/managed_components/<owner>__<name> before anything is compiled.
    root = str(CORE.relative_build_path("managed_components", idf_name))
    for sub in IDF_COMPONENT_INCLUDE_DIRS.get(name, ["include"]):
        cg.add_build_flag(f"-Wp,-I{os.path.join(root, sub)}")
    # -Wl, flags go to src's link options, so the symbols are resolved wherever the archive sits.
    for symbol in IDF_COMPONENT_LINK_SYMBOLS.get(name, []):
        cg.add_build_flag(f"-Wl,-u,{symbol}")


ESP_H264_STUB = "components/esp_h264/CMakeLists.txt"


# Runs after every other component's to_code (-1000 is CoroPriority.FINAL).
@coroutine_with_priority(-1000.0)
async def _drop_empty_esp_h264_stub():
    """Removes an empty local `esp_h264` component that would hide the real decoder.

    esp_video_camera (JPEG profile) adds an empty IDF component named esp_h264 so that esp_video's manifest
    dependency is satisfied without building H.264. IDF prefers a local component over the managed
    espressif__esp_h264 of the same name, so the real library is never built and our decoder calls fail to
    link. esp_video's own H.264 code stays Kconfig-gated, so it is unaffected by building the real one.
    """
    extra = CORE.data.get("esp32", {}).get("extra_build_files")
    if extra is not None and extra.pop(ESP_H264_STUB, None) is not None:
        # ESPHome never deletes a build file it stopped copying, so remove the copy from earlier builds.
        stale = CORE.relative_build_path("components", "esp_h264")
        if os.path.isdir(stale):
            shutil.rmtree(stale, ignore_errors=True)


async def to_code(config):
    profile = _profile()
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_URL in config:
        cg.add(var.set_url(config[CONF_URL]))
    if CONF_USERNAME in config:
        cg.add(var.set_credentials(config[CONF_USERNAME], config[CONF_PASSWORD]))
    cg.add(var.set_max_fps(config.get(CONF_MAX_FPS, profile["max_fps"])))
    if CONF_OUTPUT_WIDTH in config:
        cg.add(
            var.set_output_size(config[CONF_OUTPUT_WIDTH], config[CONF_OUTPUT_HEIGHT])
        )
    cg.add(var.set_rotation(config[CONF_ROTATION]))
    cg.add(
        var.set_max_frame_size(
            config.get(CONF_MAX_FRAME_SIZE, profile["max_frame_size"])
        )
    )
    cg.add(var.set_task_cores(profile["network_core"], profile["decode_core"]))
    cg.add(var.set_log_frame_info(config[CONF_LOG_FRAME_INFO]))
    if CONF_DISPLAY_ID in config:
        disp = await cg.get_variable(config[CONF_DISPLAY_ID])
        cg.add(var.set_display(disp))
    if CONF_LVGL_CANVAS in config:
        # LVGL creates the widget during its own setup, so pass the pointer's address, not its value.
        canvas = await cg.get_variable(config[CONF_LVGL_CANVAS])
        cg.add(var.set_lvgl_canvas(cg.RawExpression(f"&{canvas}")))
        timeout = config[CONF_HIDDEN_TIMEOUT]
        cg.add(
            var.set_hidden_timeout(
                0xFFFFFFFF if timeout == "never" else timeout.total_milliseconds
            )
        )

    if profile["idf_component"] is not None:
        name, ref = profile["idf_component"]
        _use_idf_component(name, ref)
    if profile["define"] is not None:
        cg.add_define(profile["define"])
    if config[CONF_H264]:
        # The define is global, so one stream with h264: true builds the decoder for all of them.
        name, ref = H264_IDF_COMPONENT
        _use_idf_component(name, ref)
        cg.add_define("USE_ESP_RTSP_STREAM_H264")
        if not CORE.data.get("esp_rtsp_stream_h264_stub_job"):
            CORE.data["esp_rtsp_stream_h264_stub_job"] = True
            CORE.add_job(_drop_empty_esp_h264_stub)

    # A larger TCP window keeps the camera sending while a frame is being decoded.
    _sdkconfig_default("CONFIG_LWIP_TCP_WND_DEFAULT", profile["tcp_window"])
    _sdkconfig_default("CONFIG_LWIP_TCP_RECVMBOX_SIZE", profile["tcp_recvmbox"])

    if CONF_AUDIO in config:
        audio = config[CONF_AUDIO]
        spk = await cg.get_variable(audio[CONF_SPEAKER])
        cg.add_define("USE_ESP_RTSP_STREAM_AUDIO")
        cg.add(var.set_speaker(spk))
        cg.add(var.set_audio_sample_rate(audio[CONF_SAMPLE_RATE]))
        cg.add(var.set_audio_muted(audio[CONF_MUTED]))
