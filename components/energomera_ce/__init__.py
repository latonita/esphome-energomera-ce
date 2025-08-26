import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_FLOW_CONTROL_PIN,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_PASSWORD,
)

CODEOWNERS = ["@latonita"]

MULTI_CONF = True

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_CE_ID = "ce_id"
CONF_MODEL = "model"

energomera_ce_ns = cg.esphome_ns.namespace("energomera_ce")
CEComponent = energomera_ce_ns.class_("CEComponent", cg.PollingComponent, uart.UARTDevice)

# Meter model enum
CEMeterModel = energomera_ce_ns.enum("CEMeterModel", True)
METER_MODELS = {
    "UNKNOWN": CEMeterModel.MODEL_UNKNOWN,
    "CE102": CEMeterModel.MODEL_CE102,
    "CE307": CEMeterModel.MODEL_CE307,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CEComponent),
            cv.Optional(CONF_MODEL, default="CE102"): cv.enum(METER_MODELS, upper=True),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0): cv.int_range(min=0x0, max=0xFFFF),
            cv.Optional(CONF_PASSWORD, default=0): cv.int_range(min=0x0, max=0xFFFFFFFF),
            cv.Optional(CONF_RECEIVE_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA),
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "energomera_ce", 
    require_rx=True,
    require_tx=True,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_meter_model(config[CONF_MODEL]))
    cg.add(var.set_receive_timeout(config[CONF_RECEIVE_TIMEOUT].total_milliseconds))
    cg.add(var.set_requested_meter_address(config[CONF_ADDRESS]))
    cg.add(var.set_password(config[CONF_PASSWORD]))

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

