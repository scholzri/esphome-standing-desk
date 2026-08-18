import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

desk_link_ns = cg.esphome_ns.namespace("desk_link")
DeskLink = desk_link_ns.class_("DeskLink", cg.Component)

CONF_KEYPAD_UART = "keypad_uart"
CONF_CONTROLBOX_UART = "controlbox_uart"
CONF_CAPTURE_PORT = "capture_port"
CONF_STALL_GUARD = "stall_guard"
CONF_MIN_HEIGHT = "min_height"
CONF_MAX_HEIGHT = "max_height"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DeskLink),
        cv.Required(CONF_KEYPAD_UART): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_CONTROLBOX_UART): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_CAPTURE_PORT, default=6789): cv.port,
        # Default 30 ms: comfortably above ESPHome's own ~16 ms UART drain
        # loop cadence (a frame's own wire time is well under one loop
        # period, so a held frame still incomplete well past that scale
        # means a genuine gap in delivery, not routine timing) while staying
        # below the keypad's 41 ms frame period (so the control box still
        # receives frames within one cadence period even after a flush).
        # Also more than an order of magnitude tighter than the ~1 s of
        # stream blanking that originally faulted the control box, so the
        # guard still bounds worst-case stall time by a wide margin. See the
        # matching comment on the guard check in desk_link.cpp.
        cv.Optional(CONF_STALL_GUARD, default="30ms"): cv.positive_time_period_milliseconds,
        # Feed-forward range passed to MotionController::configure() in
        # setup(). Defaults are the values measured on this desk (see
        # motion_controller.h's MotionConfig); exposed here so they can be
        # tuned from Home Assistant without a recompile.
        cv.Optional(CONF_MIN_HEIGHT, default=61.0): cv.float_,
        cv.Optional(CONF_MAX_HEIGHT, default=126.0): cv.float_,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    keypad = await cg.get_variable(config[CONF_KEYPAD_UART])
    cg.add(var.set_keypad_uart(keypad))

    controlbox = await cg.get_variable(config[CONF_CONTROLBOX_UART])
    cg.add(var.set_controlbox_uart(controlbox))

    cg.add(var.set_capture_port(config[CONF_CAPTURE_PORT]))
    cg.add(var.set_stall_guard_ms(config[CONF_STALL_GUARD]))
    cg.add(var.set_min_height_cm(config[CONF_MIN_HEIGHT]))
    cg.add(var.set_max_height_cm(config[CONF_MAX_HEIGHT]))
