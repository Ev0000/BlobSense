"""BlobSense : comptage de passages par suivi de blobs avec un VL53L5CX."""

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@ev0000"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor", "switch", "number", "button", "text_sensor"]
MULTI_CONF = True

CONF_BLOBSENSE_ID = "blobsense_id"
CONF_INT_PIN = "int_pin"
CONF_LPN_PIN = "lpn_pin"
CONF_I2C_RST_PIN = "i2c_rst_pin"
CONF_RANGING_FREQUENCY = "ranging_frequency"
CONF_INTEGRATION_TIME = "integration_time"
CONF_BACKGROUND_WINDOW = "background_window"
CONF_STARTUP_WINDOW = "startup_window"
CONF_BACKGROUND_MARGIN = "background_margin"
CONF_MIN_ACTIVE_ZONES = "min_active_zones"
CONF_MOTION_OFF_DELAY = "motion_off_delay"
CONF_AXIS = "axis"
CONF_INVERT = "invert"
CONF_BAND_WIDTH = "band_width"
CONF_PEAK_PROMINENCE = "peak_prominence"
CONF_DIRECTION_CALIBRATION_DURATION = "direction_calibration_duration"
CONF_COVER_CALIBRATION = "cover_calibration"
CONF_DISTANCE = "distance"
CONF_REFLECTANCE = "reflectance"
CONF_SAMPLES = "samples"


def _cover_distance(value):
    # « floor » : la cible est le sol, à la hauteur d'installation mesurée.
    if isinstance(value, str) and value.strip().lower() == "floor":
        return "floor"
    return cv.All(cv.distance, cv.Range(min=0.6, max=3.0))(value)


COVER_CALIBRATION_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DISTANCE, default="60cm"): _cover_distance,
        cv.Optional(CONF_REFLECTANCE, default="3%"): cv.All(
            cv.percentage, cv.Range(min=0.01, max=0.99)
        ),
        cv.Optional(CONF_SAMPLES, default=4): cv.int_range(min=1, max=16),
    }
)

blobsense_ns = cg.esphome_ns.namespace("blobsense")
BlobSenseComponent = blobsense_ns.class_(
    "BlobSenseComponent", cg.Component, i2c.I2CDevice
)
SubtractAction = blobsense_ns.class_("SubtractAction", automation.Action)

CONF_COUNTER = "counter"
CONF_VALUE = "value"


def _validate(config):
    # En 8×8, une image est faite de 4 intégrations successives, plus 1 ms de
    # surcoût (documentation ST UM2884, §4.6). Au-delà de la période de mesure,
    # le capteur réduirait de lui-même sa fréquence.
    freq = config[CONF_RANGING_FREQUENCY]
    period_ms = 1000 / freq
    integration = config[CONF_INTEGRATION_TIME].total_milliseconds
    if 4 * integration + 1 >= period_ms:
        max_integration = int((period_ms - 1) / 4 - 0.001)
        raise cv.Invalid(
            f"{CONF_INTEGRATION_TIME} trop long : en 8×8, le capteur émet 4 × {integration:.0f} ms "
            f"+ 1 ms par image, soit plus que la période de {period_ms:.0f} ms à {freq} Hz. "
            f"Maximum à {freq} Hz : {max_integration} ms (ou baisser {CONF_RANGING_FREQUENCY})."
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BlobSenseComponent),
            cv.Optional(CONF_INT_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_LPN_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_I2C_RST_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_RANGING_FREQUENCY, default=15): cv.int_range(min=1, max=15),
            cv.Optional(
                CONF_INTEGRATION_TIME, default="10ms"
            ): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=2),
                    max=cv.TimePeriod(milliseconds=1000),
                ),
            ),
            cv.Optional(CONF_BACKGROUND_WINDOW, default="10min"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=60), max=cv.TimePeriod(hours=24)),
            ),
            cv.Optional(CONF_STARTUP_WINDOW, default="10s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=2), max=cv.TimePeriod(minutes=10)),
            ),
            cv.Optional(CONF_BACKGROUND_MARGIN, default="10cm"): cv.All(
                cv.distance, cv.Range(min=0.02, max=1.0)
            ),
            cv.Optional(CONF_MIN_ACTIVE_ZONES, default=2): cv.int_range(min=1, max=64),
            cv.Optional(
                CONF_MOTION_OFF_DELAY, default="2s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_AXIS, default="y"): cv.one_of("x", "y", lower=True),
            cv.Optional(CONF_INVERT, default=False): cv.boolean,
            cv.Optional(CONF_BAND_WIDTH, default=2): cv.int_range(min=1, max=6),
            cv.Optional(CONF_DIRECTION_CALIBRATION_DURATION, default="5s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=2), max=cv.TimePeriod(seconds=60)),
            ),
            cv.Optional(CONF_COVER_CALIBRATION, default={}): COVER_CALIBRATION_SCHEMA,
            cv.Optional(CONF_PEAK_PROMINENCE, default="15cm"): cv.All(
                cv.distance, cv.Range(min=0.02, max=1.0)
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x29)),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_INT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_INT_PIN])
        cg.add(var.set_int_pin(pin))
    if CONF_LPN_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_LPN_PIN])
        cg.add(var.set_lpn_pin(pin))
    if CONF_I2C_RST_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_I2C_RST_PIN])
        cg.add(var.set_i2c_rst_pin(pin))

    cg.add(var.set_ranging_frequency(config[CONF_RANGING_FREQUENCY]))
    cg.add(var.set_integration_time(config[CONF_INTEGRATION_TIME].total_milliseconds))
    cg.add(var.set_background_window(config[CONF_BACKGROUND_WINDOW].total_milliseconds))
    cg.add(var.set_startup_window(config[CONF_STARTUP_WINDOW].total_milliseconds))
    cg.add(var.set_background_margin(int(round(config[CONF_BACKGROUND_MARGIN] * 1000))))
    cg.add(var.set_min_active_zones(config[CONF_MIN_ACTIVE_ZONES]))
    cg.add(var.set_motion_off_delay(config[CONF_MOTION_OFF_DELAY].total_milliseconds))
    cg.add(var.set_axis_x(config[CONF_AXIS] == "x"))
    cg.add(var.set_invert(config[CONF_INVERT]))
    cg.add(var.set_band_width(config[CONF_BAND_WIDTH]))
    cg.add(
        var.set_direction_calibration_duration(
            config[CONF_DIRECTION_CALIBRATION_DURATION].total_milliseconds
        )
    )
    cover = config[CONF_COVER_CALIBRATION]
    use_floor = cover[CONF_DISTANCE] == "floor"
    cg.add(
        var.set_cover_calibration(
            0 if use_floor else int(round(cover[CONF_DISTANCE] * 1000)),
            use_floor,
            int(round(cover[CONF_REFLECTANCE] * 100)),
            cover[CONF_SAMPLES],
        )
    )
    cg.add(var.set_peak_prominence(int(round(config[CONF_PEAK_PROMINENCE] * 1000))))


SUBTRACT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(BlobSenseComponent),
        cv.Required(CONF_COUNTER): cv.templatable(cv.one_of("a", "b", lower=True)),
        cv.Required(CONF_VALUE): cv.templatable(cv.int_range(min=-1998, max=1998)),
    }
)


async def subtract_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    counter = await cg.templatable(config[CONF_COUNTER], args, cg.std_string)
    cg.add(var.set_counter(counter))
    value = await cg.templatable(config[CONF_VALUE], args, cg.int32)
    cg.add(var.set_value(value))
    return var


# Le paramètre « synchronous » n'existe que dans les versions récentes d'ESPHome.
try:
    automation.register_action(
        "blobsense.subtract", SubtractAction, SUBTRACT_SCHEMA, synchronous=True
    )(subtract_to_code)
except TypeError:
    automation.register_action("blobsense.subtract", SubtractAction, SUBTRACT_SCHEMA)(
        subtract_to_code
    )
