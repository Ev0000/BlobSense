import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_BLOBSENSE_ID, BlobSenseComponent

CONF_ORIENTATION = "orientation"
CONF_COVER_CALIBRATION = "cover_calibration"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BLOBSENSE_ID): cv.use_id(BlobSenseComponent),
        # Orientation en vigueur (axis / invert) et son origine : YAML ou calibration.
        cv.Optional(CONF_ORIENTATION): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:compass-outline",
        ),
        # État de la calibration de la protection.
        cv.Optional(CONF_COVER_CALIBRATION): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:glasses",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    if conf := config.get(CONF_ORIENTATION):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(hub.set_orientation_text_sensor(sens))
    if conf := config.get(CONF_COVER_CALIBRATION):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(hub.set_cover_text_sensor(sens))
