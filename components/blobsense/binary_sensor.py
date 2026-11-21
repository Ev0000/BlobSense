import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_MOTION, DEVICE_CLASS_DOOR, DEVICE_CLASS_MOTION

from . import CONF_BLOBSENSE_ID, BlobSenseComponent

CONF_DOOR = "door"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BLOBSENSE_ID): cv.use_id(BlobSenseComponent),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOTION,
        ),
        # État de la porte déduit des mesures (après le bouton learn_door_closed) :
        # on = ouverte, off = fermée (convention Home Assistant).
        cv.Optional(CONF_DOOR): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_DOOR,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    if conf := config.get(CONF_MOTION):
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(hub.set_motion_binary_sensor(sens))
    if conf := config.get(CONF_DOOR):
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(hub.set_door_binary_sensor(sens))
