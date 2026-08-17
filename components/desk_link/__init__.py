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

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DeskLink),
        cv.Required(CONF_KEYPAD_UART): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_CONTROLBOX_UART): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_CAPTURE_PORT, default=6789): cv.port,
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
