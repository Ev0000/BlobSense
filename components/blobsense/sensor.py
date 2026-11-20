import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
)

from . import CONF_BLOBSENSE_ID, BlobSenseComponent

CONF_INSTALLATION_HEIGHT = "installation_height"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BLOBSENSE_ID): cv.use_id(BlobSenseComponent),
        cv.Optional(CONF_INSTALLATION_HEIGHT): sensor.sensor_schema(
            unit_of_measurement=UNIT_CENTIMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:arrow-expand-vertical",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    if conf := config.get(CONF_INSTALLATION_HEIGHT):
        sens = await sensor.new_sensor(conf)
        cg.add(hub.set_installation_height_sensor(sens))
