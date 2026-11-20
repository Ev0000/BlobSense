import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_BLOBSENSE_ID, BlobSenseComponent, blobsense_ns

CONF_ACTIVE = "active"
CONF_DEBUG_MATRIX = "debug_matrix"

BlobSenseSwitch = blobsense_ns.class_("BlobSenseSwitch", switch.Switch, cg.Component)
SwitchType = blobsense_ns.enum("SwitchType", is_class=True)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BLOBSENSE_ID): cv.use_id(BlobSenseComponent),
        # ALWAYS_ON par défaut : aucune écriture en flash à chaque bascule.
        cv.Optional(CONF_ACTIVE): switch.switch_schema(
            BlobSenseSwitch,
            default_restore_mode="ALWAYS_ON",
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:motion-sensor",
        ).extend(cv.COMPONENT_SCHEMA),
        cv.Optional(CONF_DEBUG_MATRIX): switch.switch_schema(
            BlobSenseSwitch,
            default_restore_mode="ALWAYS_OFF",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:grid",
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    for key, type_ in ((CONF_ACTIVE, SwitchType.ACTIVE), (CONF_DEBUG_MATRIX, SwitchType.DEBUG_MATRIX)):
        if conf := config.get(key):
            sw = await switch.new_switch(conf, type_)
            await cg.register_component(sw, conf)
            await cg.register_parented(sw, hub)
