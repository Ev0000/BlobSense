import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_BLOBSENSE_ID, BlobSenseComponent, blobsense_ns

CONF_RECALIBRATE = "recalibrate"
CONF_RESET_A = "reset_a"
CONF_RESET_B = "reset_b"

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
        cv.Optional(CONF_RESET_A): button.button_schema(
            BlobSenseButton,
            icon="mdi:counter",
        ),
        cv.Optional(CONF_RESET_B): button.button_schema(
            BlobSenseButton,
            icon="mdi:counter",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    for key, type_ in (
        (CONF_RECALIBRATE, ButtonType.RECALIBRATE),
        (CONF_RESET_A, ButtonType.RESET_A),
        (CONF_RESET_B, ButtonType.RESET_B),
    ):
        if conf := config.get(key):
            btn = await button.new_button(conf, type_)
            await cg.register_parented(btn, hub)
