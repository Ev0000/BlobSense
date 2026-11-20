import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    CONF_DISTANCE,
    CONF_UPDATE_INTERVAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    UNIT_MILLIMETER,
)

from . import CONF_BLOBSENSE_ID, BlobSenseComponent

CONF_INSTALLATION_HEIGHT = "installation_height"
CONF_ZONE = "zone"

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
        # Distance brute d'une zone (par défaut la zone 27, au centre de la
        # matrice), moyennée sur update_interval. Pour les essais et la mesure
        # de distance simple. Plusieurs blocs « platform: blobsense » peuvent
        # chacun déclarer une distance sur une zone différente.
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:ruler",
        ).extend(
            {
                cv.Optional(CONF_ZONE, default=27): cv.int_range(min=0, max=63),
                cv.Optional(
                    CONF_UPDATE_INTERVAL, default="1s"
                ): cv.positive_time_period_milliseconds,
            }
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    if conf := config.get(CONF_INSTALLATION_HEIGHT):
        sens = await sensor.new_sensor(conf)
        cg.add(hub.set_installation_height_sensor(sens))
    if conf := config.get(CONF_DISTANCE):
        sens = await sensor.new_sensor(conf)
        cg.add(
            hub.add_distance_sensor(
                sens, conf[CONF_ZONE], conf[CONF_UPDATE_INTERVAL].total_milliseconds
            )
        )
