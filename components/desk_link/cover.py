import esphome.codegen as cg
from esphome.components import cover
import esphome.config_validation as cv

from . import CONF_DESK_LINK_ID, DeskLink, desk_link_ns

DEPENDENCIES = ["desk_link"]

DeskCover = desk_link_ns.class_("DeskCover", cover.Cover, cg.Component)

# device_class is intentionally omitted: ESPHome's cover DEVICE_CLASSES
# (awning, blind, curtain, damper, door, garage, gate, shade, shutter,
# window) has no "desk"/furniture entry, and "shade" is the closest visual
# analogue but implies a window covering in HA's UI in a way a standing desk
# is not. Home Assistant already renders a position+stop cover sensibly with
# no device_class set, so omitting it (matching what cover_schema() does when
# no device_class default is passed) was chosen over picking a misleading
# best-fit label.
CONFIG_SCHEMA = (
    cover.cover_schema(DeskCover)
    .extend(
        {
            cv.GenerateID(CONF_DESK_LINK_ID): cv.use_id(DeskLink),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_DESK_LINK_ID])
    cg.add(var.set_desk_link(parent))
