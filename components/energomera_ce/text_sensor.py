import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    CONF_DATE,
    CONF_TIME,
    CONF_DATETIME,
    CONF_NAME,
)

from . import CEComponent, CONF_CE_ID

AUTO_LOAD = ["energomera_ce"]
CODEOWNERS = ["@latonita"]

CONF_ELECTRO_TARIFF = "electricity_tariff"

CONF_NETWORK_ADDRESS = "network_address"
CONF_SERIAL_NR = "serial_nr"
CONF_READING_STATE = "reading_state"
CONF_ERROR_CODE = "error_code"
CONF_ABOUT = "about"

TEXT_SENSORS = [
#    CONF_ELECTRO_TARIFF,
    CONF_DATE,
    CONF_TIME,
    CONF_DATETIME,
    CONF_NETWORK_ADDRESS,
    CONF_SERIAL_NR,
    CONF_READING_STATE,
    CONF_ERROR_CODE,
    CONF_ABOUT,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CE_ID): cv.use_id(CEComponent),
        # cv.Optional(CONF_ELECTRO_TARIFF): cv.maybe_simple_value(
        #     text_sensor.text_sensor_schema(),
        #     key=CONF_NAME,
        # ),
        cv.Optional(CONF_DATE): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_TIME): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_DATETIME): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_NETWORK_ADDRESS): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_SERIAL_NR): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_READING_STATE): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_ERROR_CODE): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_ABOUT): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_CE_ID])

    for key in TEXT_SENSORS:
        if key not in config:
            continue
        sensor_conf = config[key]
        text_sens = await text_sensor.new_text_sensor(sensor_conf)
        cg.add(getattr(hub, f"set_{key}_text_sensor")(text_sens))
