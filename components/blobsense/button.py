import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_BLOBSENSE_ID, BlobSenseComponent, blobsense_ns

CONF_RECALIBRATE = "recalibrate"

BlobSenseButton = blobsense_ns.class_("BlobSenseButton", button.Button)
ButtonType = blobsense_ns.enum("ButtonType", is_class=True)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BLOBSENSE_ID): cv.use_id(BlobSenseComponent),
        cv.Optional(CONF_RECALIBRATE): button.button_schema(
            BlobSenseButton,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:refresh",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    if conf := config.get(CONF_RECALIBRATE):
        btn = await button.new_button(conf, ButtonType.RECALIBRATE)
        await cg.register_parented(btn, hub)
