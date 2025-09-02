import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import esp32, display
from esphome import automation

DEPENDENCIES = ["esp32", "wifi"]

live_ns = cg.esphome_ns.namespace("live")
LiveComponent = live_ns.class_("LiveComponent", cg.Component)

# Configuration
CONF_RTSP_URL = "rtsp_url"
CONF_BUFFER_SIZE = "buffer_size"
CONF_TIMEOUT = "timeout"
CONF_TARGET_FPS = "target_fps"
CONF_DECODE_FORMAT = "decode_format"
CONF_ENABLE_HARDWARE_DECODE = "enable_hardware_decode"
CONF_DISPLAY_AREA = "display_area"
CONF_LCD_PANEL = "lcd_panel"

# Callbacks
CONF_ON_FRAME = "on_frame"
CONF_ON_CONNECT = "on_connect"
CONF_ON_DISCONNECT = "on_disconnect"

# Schéma pour la zone d'affichage
DISPLAY_AREA_SCHEMA = cv.Schema({
    cv.Optional("x", default=0): cv.positive_int,
    cv.Optional("y", default=0): cv.positive_int,
    cv.Required("width"): cv.positive_int,
    cv.Required("height"): cv.positive_int,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LiveComponent),
    cv.Required(CONF_RTSP_URL): cv.string,
    cv.Optional(CONF_BUFFER_SIZE, default=8192): cv.positive_int,
    cv.Optional(CONF_TIMEOUT, default=5000): cv.positive_int,
    cv.Optional(CONF_TARGET_FPS, default=25): cv.int_range(min=1, max=60),
    cv.Optional(CONF_DECODE_FORMAT, default="h264"): cv.one_of("h264", "jpeg", "mjpeg", lower=True),
    cv.Optional(CONF_ENABLE_HARDWARE_DECODE, default=True): cv.boolean,
    cv.Optional(CONF_DISPLAY_AREA): DISPLAY_AREA_SCHEMA,
    cv.Optional(CONF_LCD_PANEL): cv.use_id(display.DisplayBuffer),
    
    # Callbacks - using automation triggers
    cv.Optional(CONF_ON_FRAME): automation.validate_automation({
        cv.GenerateID(): cv.declare_id(automation.Trigger.template(live_ns.struct("VideoFrame").operator("const").operator("ref")))
    }),
    cv.Optional(CONF_ON_CONNECT): automation.validate_automation({
        cv.GenerateID(): cv.declare_id(automation.Trigger.template())
    }),
    cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation({
        cv.GenerateID(): cv.declare_id(automation.Trigger.template())
    }),
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
    
    # Configuration de l'affichage
    if CONF_DISPLAY_AREA in config:
        area = config[CONF_DISPLAY_AREA]
        cg.add(var.set_display_area(area["x"], area["y"], area["width"], area["height"]))
        
    if CONF_LCD_PANEL in config:
        panel = await cg.get_variable(config[CONF_LCD_PANEL])
        cg.add(var.set_lcd_panel(panel))
    
    # Callbacks - setup automation triggers
    if CONF_ON_FRAME in config:
        for conf in config[CONF_ON_FRAME]:
            trigger = cg.new_Pvariable(conf[CONF_ID])
            cg.add(var.set_on_frame_trigger(trigger))
            await automation.build_automation(trigger, [(live_ns.struct("VideoFrame").operator("const").operator("ref"), "frame")], conf)
            
    if CONF_ON_CONNECT in config:
        for conf in config[CONF_ON_CONNECT]:
            trigger = cg.new_Pvariable(conf[CONF_ID])
            cg.add(var.set_on_connect_trigger(trigger))
            await automation.build_automation(trigger, [], conf)
            
    if CONF_ON_DISCONNECT in config:
        for conf in config[CONF_ON_DISCONNECT]:
            trigger = cg.new_Pvariable(conf[CONF_ID])
            cg.add(var.set_on_disconnect_trigger(trigger))
            await automation.build_automation(trigger, [], conf)
    
    # Dépendances ESP-IDF pour le hardware
    cg.add_platformio_option("lib_deps", [
        "esp32-jpeg-decoder",
    ])
    
    # Flags de compilation pour activer les accélérateurs hardware
    cg.add_build_flag("-DCONFIG_ESP32P4_JPEG_DECODE_ACCELERATE=1")
    cg.add_build_flag("-DCONFIG_ESP32P4_H264_DECODE_ACCELERATE=1") 
    cg.add_build_flag("-DCONFIG_ESP32P4_PPA_ACCELERATE=1")
    cg.add_build_flag("-DCONFIG_ESP32_WIFI_ENABLED=1")
    cg.add_build_flag("-DCONFIG_LWIP_PPP_SUPPORT=1")
    
    # Includes ESP-IDF
    cg.add_global(cg.RawExpression('#include "esp_jpeg_dec.h"'))
    cg.add_global(cg.RawExpression('#include "esp_h264_dec.h"'))
    cg.add_global(cg.RawExpression('#include "esp_ppa.h"'))
    cg.add_global(cg.RawExpression('#include "esp_lcd_panel_ops.h"'))
