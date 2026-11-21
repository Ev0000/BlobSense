import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_INITIAL_VALUE,
    CONF_RESTORE_VALUE,
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CENTIMETER,
)

from esphome.core import CORE

from . import CONF_BLOBSENSE_ID, BlobSenseComponent, blobsense_ns

CONF_MIN_HEIGHT = "min_height"
CONF_MAX_HEIGHT = "max_height"
CONF_INTEGRATION_TIME = "integration_time"

BlobSenseNumber = blobsense_ns.class_("BlobSenseNumber", number.Number, cg.Component)
NumberType = blobsense_ns.enum("NumberType", is_class=True)


def _height_schema(default):
    return (
        number.number_schema(
            BlobSenseNumber,
            unit_of_measurement=UNIT_CENTIMETER,
            device_class=DEVICE_CLASS_DISTANCE,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:human-male-height",
        )
        .extend(
            {
                cv.Optional(CONF_INITIAL_VALUE, default=default): cv.float_range(
                    min=0, max=300
                ),
                cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
            }
        )
        .extend(cv.COMPONENT_SCHEMA)
    )


# Temps d'intégration réglable à chaud (ms). Sa valeur initiale est celle du
# YAML du composant ; la valeur choisie est conservée après redémarrage.
INTEGRATION_SCHEMA = (
    number.number_schema(
        BlobSenseNumber,
        unit_of_measurement="ms",
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:timer-cog-outline",
    )
    .extend(
        {
            cv.Optional(CONF_INITIAL_VALUE): cv.int_range(min=2, max=250),
            cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BLOBSENSE_ID): cv.use_id(BlobSenseComponent),
        cv.Optional(CONF_MIN_HEIGHT): _height_schema(100),
        cv.Optional(CONF_MAX_HEIGHT): _height_schema(210),
        cv.Optional(CONF_INTEGRATION_TIME): INTEGRATION_SCHEMA,
    }
)


def _hub_config(hub_id):
    for conf in CORE.config.get("blobsense", []):
        if conf["id"] == hub_id:
            return conf
    return {}


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BLOBSENSE_ID])
    for key, type_ in ((CONF_MIN_HEIGHT, NumberType.MIN_HEIGHT), (CONF_MAX_HEIGHT, NumberType.MAX_HEIGHT)):
        if conf := config.get(key):
            num = await number.new_number(conf, type_, min_value=0, max_value=300, step=1)
            await cg.register_component(num, conf)
            await cg.register_parented(num, hub)
            cg.add(num.set_initial_value(conf[CONF_INITIAL_VALUE]))
            cg.add(num.set_restore_value(conf[CONF_RESTORE_VALUE]))
        else:
            # Valeur par défaut si l'entité n'est pas déclarée.
            setter = hub.set_min_height if key == CONF_MIN_HEIGHT else hub.set_max_height
            cg.add(setter(100 if key == CONF_MIN_HEIGHT else 210))

    if conf := config.get(CONF_INTEGRATION_TIME):
        hub_conf = _hub_config(config[CONF_BLOBSENSE_ID])
        freq = hub_conf.get("ranging_frequency", 15)
        max_ms = int((1000 / freq - 1) / 4)
        yaml_ms = hub_conf.get("integration_time")
        initial = conf.get(
            CONF_INITIAL_VALUE,
            int(yaml_ms.total_milliseconds) if yaml_ms is not None else 10,
        )
        num = await number.new_number(
            conf, NumberType.INTEGRATION_TIME, min_value=2, max_value=max_ms, step=1
        )
        await cg.register_component(num, conf)
        await cg.register_parented(num, hub)
        cg.add(num.set_initial_value(min(initial, max_ms)))
        cg.add(num.set_restore_value(conf[CONF_RESTORE_VALUE]))
