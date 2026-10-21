import esphome.codegen as cg
from esphome.components import display, esp32
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
            cv.Optional(CONF_LOG_FRAME_INFO, default=True): cv.boolean,
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

    if profile["idf_component"] is not None:
        name, ref = profile["idf_component"]
        esp32.add_idf_component(name=name, ref=ref)
    if profile["define"] is not None:
        cg.add_define(profile["define"])

    # A larger TCP window keeps the camera sending while a frame is being decoded.
    _sdkconfig_default("CONFIG_LWIP_TCP_WND_DEFAULT", profile["tcp_window"])
    _sdkconfig_default("CONFIG_LWIP_TCP_RECVMBOX_SIZE", profile["tcp_recvmbox"])
