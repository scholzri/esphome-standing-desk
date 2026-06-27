import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart, sensor
from esphome.const import CONF_ID

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor']

desk_keypad_ns = cg.esphome_ns.namespace('desk_keypad')
DeskKeypad = desk_keypad_ns.class_('DeskKeypad', cg.Component)
MoveToTargetAction = desk_keypad_ns.class_('MoveToTargetAction', automation.Action)

CONF_KEYPAD_UART = 'keypad_uart'
CONF_CONTROLBOX_UART = 'controlbox_uart'
CONF_INJECTION_INTERVAL = 'injection_interval'
CONF_HEIGHT_SENSOR = 'height_sensor'
CONF_TARGET = 'target'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DeskKeypad),
    cv.Required(CONF_KEYPAD_UART): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_CONTROLBOX_UART): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_INJECTION_INTERVAL, default=100): cv.positive_time_period_milliseconds,
    cv.Required(CONF_HEIGHT_SENSOR): cv.use_id(sensor.Sensor),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for uart_type in [CONF_KEYPAD_UART, CONF_CONTROLBOX_UART]:
        uart_var = await cg.get_variable(config[uart_type])
        cg.add(getattr(var, f"set_{uart_type}")(uart_var))

    cg.add(var.set_injection_interval(config[CONF_INJECTION_INTERVAL]))
    height_sensor = await cg.get_variable(config[CONF_HEIGHT_SENSOR])
    cg.add(var.set_height_sensor(height_sensor))

    cg.add_global(desk_keypad_ns.using)
    cg.add_define("USE_DESK_KEYPAD")


MOVE_TO_TARGET_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(DeskKeypad),
    cv.Required(CONF_TARGET): cv.templatable(cv.float_),
})


@automation.register_action("desk_keypad.move_to_target", MoveToTargetAction, MOVE_TO_TARGET_SCHEMA)
async def move_to_target_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_TARGET], args, float)
    cg.add(var.set_target(template_))
    return var
