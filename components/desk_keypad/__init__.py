import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor']

desk_keypad_ns = cg.esphome_ns.namespace('desk_keypad')
DeskKeypad = desk_keypad_ns.class_('DeskKeypad', cg.Component)

CONF_KEYPAD_UART = 'keypad_uart'
CONF_CONTROLBOX_UART = 'controlbox_uart'
CONF_INJECTION_INTERVAL = 'injection_interval'
CONF_HEIGHT_SENSOR = 'height_sensor'

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
