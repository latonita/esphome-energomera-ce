import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_ACTIVE_POWER,
    CONF_CURRENT,
    CONF_NAME,
    CONF_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    ICON_POWER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_VOLT,
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

CONF_VOLTAGE_A = "voltage_a"
CONF_VOLTAGE_B = "voltage_b"
CONF_VOLTAGE_C = "voltage_c"

CONF_CURRENT_A = "current_a"
CONF_CURRENT_B = "current_b"
CONF_CURRENT_C = "current_c"

CONF_ACTIVE_POWER_A = "active_power_a"
CONF_ACTIVE_POWER_B = "active_power_b"
CONF_ACTIVE_POWER_C = "active_power_c"

TARIFF_CONSUMPTION_SCHEMA = cv.maybe_simple_value(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        icon="mdi:transmission-tower-export",
    ),
    key=CONF_NAME,
)

VOLTAGE_SCHEMA = cv.maybe_simple_value(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_VOLTAGE,
    ),
    key=CONF_NAME,
)

CURRENT_SCHEMA = cv.maybe_simple_value(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    key=CONF_NAME,
)

ACTIVE_POWER_SCHEMA = cv.maybe_simple_value(
    sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
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
        
        cv.Optional(CONF_VOLTAGE): VOLTAGE_SCHEMA,
        cv.Optional(CONF_VOLTAGE_A): VOLTAGE_SCHEMA,
        cv.Optional(CONF_VOLTAGE_B): VOLTAGE_SCHEMA,
        cv.Optional(CONF_VOLTAGE_C): VOLTAGE_SCHEMA,

        cv.Optional(CONF_ACTIVE_POWER): ACTIVE_POWER_SCHEMA,
        cv.Optional(CONF_ACTIVE_POWER_A): ACTIVE_POWER_SCHEMA,
        cv.Optional(CONF_ACTIVE_POWER_B): ACTIVE_POWER_SCHEMA,
        cv.Optional(CONF_ACTIVE_POWER_C): ACTIVE_POWER_SCHEMA,

        cv.Optional(CONF_CURRENT): CURRENT_SCHEMA,
        cv.Optional(CONF_CURRENT_A): CURRENT_SCHEMA,
        cv.Optional(CONF_CURRENT_B): CURRENT_SCHEMA,
        cv.Optional(CONF_CURRENT_C): CURRENT_SCHEMA,
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
    
    if conf_voltage := config.get(CONF_VOLTAGE):
        sens = await sensor.new_sensor(conf_voltage)
        cg.add(getattr(hub, f"set_voltage_sensor")(sens))

    if conf_voltage_a := config.get(CONF_VOLTAGE_A):
        sens = await sensor.new_sensor(conf_voltage_a)
        cg.add(getattr(hub, f"set_voltage_a_sensor")(sens))

    if conf_voltage_b := config.get(CONF_VOLTAGE_B):
        sens = await sensor.new_sensor(conf_voltage_b)
        cg.add(getattr(hub, f"set_voltage_b_sensor")(sens))

    if conf_voltage_c := config.get(CONF_VOLTAGE_C):
        sens = await sensor.new_sensor(conf_voltage_c)
        cg.add(getattr(hub, f"set_voltage_c_sensor")(sens))

    if conf_active_power := config.get(CONF_ACTIVE_POWER):
        sens = await sensor.new_sensor(conf_active_power)
        cg.add(getattr(hub, f"set_power_sensor")(sens))

    if conf_active_power_a := config.get(CONF_ACTIVE_POWER_A):
        sens = await sensor.new_sensor(conf_active_power_a)
        cg.add(getattr(hub, f"set_power_a_sensor")(sens))

    if conf_active_power_b := config.get(CONF_ACTIVE_POWER_B):
        sens = await sensor.new_sensor(conf_active_power_b)
        cg.add(getattr(hub, f"set_power_b_sensor")(sens))

    if conf_active_power_c := config.get(CONF_ACTIVE_POWER_C):
        sens = await sensor.new_sensor(conf_active_power_c)
        cg.add(getattr(hub, f"set_power_c_sensor")(sens))

    if conf_current := config.get(CONF_CURRENT):
        sens = await sensor.new_sensor(conf_current)
        cg.add(getattr(hub, f"set_current_sensor")(sens))

    if conf_current_a := config.get(CONF_CURRENT_A):
        sens = await sensor.new_sensor(conf_current_a)
        cg.add(getattr(hub, f"set_current_a_sensor")(sens))

    if conf_current_b := config.get(CONF_CURRENT_B):
        sens = await sensor.new_sensor(conf_current_b)
        cg.add(getattr(hub, f"set_current_b_sensor")(sens))

    if conf_current_c := config.get(CONF_CURRENT_C):
        sens = await sensor.new_sensor(conf_current_c)
        cg.add(getattr(hub, f"set_current_c_sensor")(sens))