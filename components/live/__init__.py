import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import esp32

DEPENDENCIES = ["esp32", "wifi"]
CODEOWNERS = ["@yourusername"]

live_ns = cg.esphome_ns.namespace("live")
LiveComponent = live_ns.class_("LiveComponent", cg.Component)

# Configuration options
CONF_RTSP_URL = "rtsp_url"
CONF_BUFFER_SIZE = "buffer_size"
CONF_TIMEOUT = "timeout"
CONF_TARGET_FPS = "target_fps"
CONF_DECODE_FORMAT = "decode_format"
CONF_ENABLE_HARDWARE_DECODE = "enable_hardware_decode"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LiveComponent),
    cv.Required(CONF_RTSP_URL): cv.string,
    cv.Optional(CONF_BUFFER_SIZE, default=8192): cv.positive_int,
    cv.Optional(CONF_TIMEOUT, default=5000): cv.positive_int,
    cv.Optional(CONF_TARGET_FPS, default=25): cv.int_range(min=1, max=60),
    cv.Optional(CONF_DECODE_FORMAT, default="h264"): cv.one_of("h264", "jpeg", "mjpeg", lower=True),
    cv.Optional(CONF_ENABLE_HARDWARE_DECODE, default=True): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Configuration de base
    cg.add(var.set_rtsp_url(config[CONF_RTSP_URL]))
    cg.add(var.set_buffer_size(config[CONF_BUFFER_SIZE]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_target_fps(config[CONF_TARGET_FPS]))
    cg.add(var.set_decode_format(config[CONF_DECODE_FORMAT]))
    cg.add(var.enable_hardware_decoding(config[CONF_ENABLE_HARDWARE_DECODE]))
    
    # Dépendances de bibliothèques pour ESP32-P4
    cg.add_platformio_option("lib_deps", [
        "https://github.com/espressif/esp-h264-component.git",
        "https://github.com/espressif/esp-video-components.git",
    ])
    
    # Build flags pour ESP32-P4
    cg.add_build_flag("-DCONFIG_ESP32_WIFI_ENABLED=1")
    cg.add_build_flag("-DCONFIG_LWIP_PPP_SUPPORT=1")
    
    # Support pour ESP32-P4 hardware acceleration
    cg.add_define("ESP32_P4_HARDWARE_DECODE_SUPPORT")
