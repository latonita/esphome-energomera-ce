import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_ACTIVE_POWER,
    CONF_NAME,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    ICON_POWER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)
from . import CEComponent, CONF_CE_ID

CODEOWNERS = ["@latonita"]

DEPENDENCIES = ["energomera_ce"]

ICON_VOLTAGE = "mdi:sine-wave"

CONF_TARIFF_1 = "tariff_1"
CONF_TARIFF_2 = "tariff_2"
CONF_TARIFF_3 = "tariff_3"
CONF_TARIFF_4 = "tariff_4"


TARIFF_CONSUMPTION_SCHEMA = cv.maybe_simple_value(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT_HOURS,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        icon="mdi:transmission-tower-export",
    ),
    key=CONF_NAME,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CE_ID): cv.use_id(CEComponent),
        cv.Optional(CONF_TARIFF_1): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_2): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_3): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_4): TARIFF_CONSUMPTION_SCHEMA,
        # CONF_ACTIVE_POWER: cv.maybe_simple_value(
        #     sensor.sensor_schema(
        #         unit_of_measurement=UNIT_WATT,
        #         accuracy_decimals=0,
        #         device_class=DEVICE_CLASS_POWER,
        #         state_class=STATE_CLASS_MEASUREMENT,
        #         icon=ICON_POWER,
        #     ),
        #     key=CONF_NAME,
        # ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_CE_ID])

    for i, tariff_key in enumerate([CONF_TARIFF_1, CONF_TARIFF_2, CONF_TARIFF_3,
                                     CONF_TARIFF_4], start=1):
        if tariff_key not in config:
            continue
        sens = await sensor.new_sensor(config[tariff_key])
        cg.add(getattr(hub, f"set_tariff_sensor")(i, sens))
