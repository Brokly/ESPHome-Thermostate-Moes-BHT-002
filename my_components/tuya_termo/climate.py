import logging
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import climate, uart, sensor, binary_sensor, text_sensor, number, select, switch, time, lock
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_INTERNAL,
    CONF_OPTIMISTIC,
    CONF_DATA,
    CONF_NUMBER,
    CONF_SUPPORTED_MODES,
    CONF_SUPPORTED_SWING_MODES,
    CONF_SUPPORTED_PRESETS,
    CONF_TIME_ID,
    UNIT_CELSIUS,
    ICON_POWER,
    ICON_THERMOMETER,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    CONF_TYPE,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    CONF_ICON,
    CONF_MODE,
    CONF_MAX_TEMPERATURE,
    CONF_MIN_TEMPERATURE,
    CONF_TEMPERATURE_STEP,
    CONF_VISUAL,
    CONF_HOURS,
    CONF_MINUTES,
    CONF_TEMPERATURE,
)

from esphome.components.climate import (
    ClimateMode,
    ClimatePreset,
)

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@Brokly"]
DEPENDENCIES = ["climate", "uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor", "number", "select", "switch", "lock", "time"]

CONF_INTERNAL_TEMPERATURE = 'internal_temperature'
CONF_EXTERNAL_TEMPERATURE = 'external_temperature'
CONF_CHILDREN_LOCK = 'children_lock'
ICON_CHILDREN_LOCK = 'mdi:lock'

CONF_SHEDULE = 'shedule'
CONF_SHEDULE_HOURS = CONF_HOURS
ICON_SHEDULE_HOURS = 'mdi:timer-play'
CONF_SHEDULE_MINUTES = CONF_MINUTES
ICON_SHEDULE_MINUTES = 'mdi:timer-play-outline'
CONF_SHEDULE_TEMPERATURE = CONF_TEMPERATURE
CONF_SHEDULE_SELECTOR = 'selector'
ICON_SHEDULE_SELECTOR = 'mdi:calendar-clock'

CONF_PRODUCT_ID = 'product_id'
ICON_PRODUCT_ID = 'mdi:cog'
CONF_ECO_TEMPERATURE= 'eco_temperature'
CONF_OVERHEAT_TEMPERATURE= 'overheat_temperature'
CONF_DEADZONE_TEMPERATURE= 'deadzone_temperature'
CONF_INPUT_RESET_PIN= 'reset_pin' # вход переинициализации протокола обмена 
CONF_MCU_RESET_PIN= 'mcu_reset_pin' # выход управления перезагрузкой процессора термостата
CONF_STATUS_PIN= 'status_pin' # выход светодиода статуса WIFI 
CONF_MODE_RESTORE= 'mode_restore' # восстанавливать режим работы после перезагрузки (true/false)
CONF_TIME_SYNC_MARKS= 'time_sync_packets' # регулярные пакеты синхронизации времени (true/false)
CONF_MCU_RELOAD_COUNTER= 'mcu_reload_counter'
ICON_MCU_RELOAD_COUNTER= 'mdi:cog-counterclockwise'

tuya_termo_ns = cg.esphome_ns.namespace("tuya_termo")
TuyaTermo = tuya_termo_ns.class_("TuyaTermo", climate.Climate, cg.Component)
TuyaTermo_Switch = tuya_termo_ns.class_("TuyaTermo_Switch", switch.Switch, cg.Component)
TuyaTermo_Number = tuya_termo_ns.class_("TuyaTermo_Number", number.Number, cg.Component)
TuyaTermo_Select = tuya_termo_ns.class_("TuyaTermo_Select", select.Select, cg.Component)
#TuyaTermo_Lock = tuya_termo_ns.class_("TuyaTermo_Lock", lock.Lock, cg.Component)
#check_plan = 0

ALLOWED_CLIMATE_MODES = {
    "AUTO": ClimateMode.CLIMATE_MODE_AUTO,
    "HEAT": ClimateMode.CLIMATE_MODE_HEAT,
}
validate_modes = cv.enum(ALLOWED_CLIMATE_MODES, upper=True)

ALLOWED_CLIMATE_PRESETS = {
    "ECO": ClimatePreset.CLIMATE_PRESET_ECO,
}
validate_presets = cv.enum(ALLOWED_CLIMATE_PRESETS, upper=True)

def validate_pins(config):
    if CONF_INPUT_RESET_PIN in config and CONF_MCU_RESET_PIN in config:
        raise cv.Invalid(f"Options 'mcu_reset_pin' and 'input_reset_pin' cannot be used simultaneously.")
    return config

def validate_plan(config):
    if CONF_SHEDULE in config:
       shedule = config[CONF_SHEDULE]
       if CONF_SHEDULE in config or CONF_SHEDULE_SELECTOR in shedule or CONF_SHEDULE_TEMPERATURE in shedule or CONF_SHEDULE_MINUTES in shedule or CONF_SHEDULE_HOURS in shedule:
           if CONF_SHEDULE_SELECTOR not in shedule or CONF_SHEDULE_TEMPERATURE not in shedule or CONF_SHEDULE_MINUTES not in shedule or CONF_SHEDULE_HOURS not in shedule:
              raise cv.Invalid(f"Section 'shedule' must have options 'selector', 'hours', 'minutes' and 'temperature'.")
    return config

NumberMode = tuya_termo_ns.enum("NumberMode")

NUMBER_MODES = {
    "AUTO": NumberMode.NUMBER_MODE_AUTO,
    "BOX": NumberMode.NUMBER_MODE_BOX,
    "SLIDER": NumberMode.NUMBER_MODE_SLIDER,
}

def output_info(config):
    """_LOGGER.info(config)"""
    return config

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TuyaTermo),
            cv.Optional(CONF_OPTIMISTIC, default="true"): cv.boolean,
            cv.Optional(CONF_VISUAL, default={}): cv.Schema(
              {
                cv.Optional(CONF_MIN_TEMPERATURE, default=5.0): cv.temperature,
                cv.Optional(CONF_MAX_TEMPERATURE, default=35.0): cv.temperature,
                #cv.Optional(CONF_TEMPERATURE_STEP): cv.float_with_unit(
                #    "visual_temperature", "(°C|° C|°|C|° K|° K|K|°F|° F|F)?"
                #),
                cv.Optional(CONF_ECO_TEMPERATURE, default=20.0): cv.temperature,
                cv.Optional(CONF_OVERHEAT_TEMPERATURE, default=45.0): cv.temperature,
                cv.Optional(CONF_DEADZONE_TEMPERATURE, default=1.0): cv.float_range(
                    min=1.0, max=5.0
                ),
              }
            ),
            cv.Optional(CONF_SHEDULE, default={}): cv.Schema(
              {
                cv.Optional(CONF_SHEDULE_SELECTOR): select.SELECT_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend(
                  {
                     cv.GenerateID(): cv.declare_id(TuyaTermo_Select),
                     cv.Optional(CONF_ICON, default=ICON_SHEDULE_SELECTOR): cv.icon,
                  },
                ),
                cv.Optional(CONF_SHEDULE_HOURS): number.NUMBER_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend(
                  {
                     cv.GenerateID(): cv.declare_id(TuyaTermo_Number),
                     cv.Optional(CONF_ICON, default=ICON_SHEDULE_HOURS): cv.icon,
                     cv.Optional(CONF_MODE, default="BOX"): cv.enum(NUMBER_MODES, upper=True),
                  },
                ),
                cv.Optional(CONF_SHEDULE_MINUTES): number.NUMBER_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend(
                  {
                     cv.GenerateID(): cv.declare_id(TuyaTermo_Number),
                     cv.Optional(CONF_ICON, default=ICON_SHEDULE_MINUTES): cv.icon,
                     cv.Optional(CONF_MODE, default="BOX"): cv.enum(NUMBER_MODES, upper=True),
                  },
                ),
                cv.Optional(CONF_SHEDULE_TEMPERATURE): number.NUMBER_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend(
                  {
                     cv.GenerateID(): cv.declare_id(TuyaTermo_Number),
                     cv.Optional(CONF_ICON, default=ICON_THERMOMETER): cv.icon,
                  },
                ),
              }            
            ), 

            # нога входного сигнала сброса
            cv.Optional(CONF_INPUT_RESET_PIN ): pins.gpio_input_pin_schema,
            # выходная нога ресета MCU термостата
            cv.Optional(CONF_MCU_RESET_PIN ): pins.gpio_output_pin_schema,
            # выходная нога отметок синхронизации времени
            cv.Optional(CONF_STATUS_PIN ): pins.gpio_output_pin_schema,
            # счетчик перезагрузок
            cv.Optional(CONF_MCU_RELOAD_COUNTER): sensor.sensor_schema(
               accuracy_decimals=1,
               state_class=STATE_CLASS_MEASUREMENT,
               entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
               icon=ICON_MCU_RELOAD_COUNTER,
            ),
            # источник синхронизации времени
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_INTERNAL_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_EXTERNAL_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MODE_RESTORE, default=True): cv.boolean,
            cv.Optional(CONF_TIME_SYNC_MARKS, default=False): cv.boolean,
            cv.Optional(CONF_SUPPORTED_MODES): cv.ensure_list(validate_modes),
            cv.Optional(CONF_SUPPORTED_PRESETS): cv.ensure_list(validate_presets),
            cv.Optional(CONF_CHILDREN_LOCK): switch.switch_schema(
               TuyaTermo_Switch,
               icon=ICON_CHILDREN_LOCK,
            ),
            cv.Optional(CONF_PRODUCT_ID): text_sensor.text_sensor_schema(
               icon=ICON_PRODUCT_ID,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    output_info,
    validate_pins,
    validate_plan,
)

async def to_code(config):
    """_LOGGER.info("--------------")"""
    """_LOGGER.info(config)"""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    
    if CONF_UART_ID in config:
       parent = await cg.get_variable(config[CONF_UART_ID])
       cg.add(var.initTermo(parent))
    else:
       raise cv.Invalid(
          f"Setting 'uart_id' is required !"
       )
    
    cg.add(var.set_optimistic(config[CONF_OPTIMISTIC]))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(time_))

    if CONF_EXTERNAL_TEMPERATURE in config:
        conf = config[CONF_EXTERNAL_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_external_temperature_sensor(sens))
    
    if CONF_INTERNAL_TEMPERATURE in config:
        conf = config[CONF_INTERNAL_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_internal_temperature_sensor(sens))

    if CONF_SUPPORTED_MODES in config:
        cg.add(var.set_supported_modes(config[CONF_SUPPORTED_MODES]))
    
    if CONF_SUPPORTED_PRESETS in config:
        cg.add(var.set_supported_presets(config[CONF_SUPPORTED_PRESETS]))

    if CONF_CHILDREN_LOCK in config:
        cg.add(var.set_children_lock_switch(await switch.new_switch(config[CONF_CHILDREN_LOCK])))

    if CONF_PRODUCT_ID in config:
        cg.add(var.set_product_id_text(await text_sensor.new_text_sensor(config[CONF_PRODUCT_ID])))

    visual = config[CONF_VISUAL]
    if CONF_MIN_TEMPERATURE in visual:
        cg.add(var.set_visual_min_temperature_override(visual[CONF_MIN_TEMPERATURE]))
    if CONF_MAX_TEMPERATURE in visual:
        cg.add(var.set_visual_max_temperature_override(visual[CONF_MAX_TEMPERATURE]))
    #if CONF_TEMPERATURE_STEP in visual:
    #    cg.add(var.set_visual_temperature_step_override(visual[CONF_TEMPERATURE_STEP]))
    if CONF_ECO_TEMPERATURE in visual:
        cg.add(var.set_visual_temperature_eco(visual[CONF_ECO_TEMPERATURE]))
    if CONF_OVERHEAT_TEMPERATURE in visual:
        cg.add(var.set_visual_temperature_overheat(visual[CONF_OVERHEAT_TEMPERATURE]))
    if CONF_DEADZONE_TEMPERATURE in visual:
        cg.add(var.set_visual_temperature_deadzone(visual[CONF_DEADZONE_TEMPERATURE]))
    if CONF_INPUT_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_INPUT_RESET_PIN])
        cg.add(var.set_input_reset_pin(pin))
    if CONF_MCU_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_MCU_RESET_PIN])
        cg.add(var.set_mcu_reset_pin(pin))
        cg.add_define("USE_OTA_STATE_CALLBACK")
    if CONF_STATUS_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_STATUS_PIN])
        cg.add(var.set_status_pin(pin))
    if CONF_MODE_RESTORE in config:
        cg.add(var.set_mode_restore(config[CONF_MODE_RESTORE]))
    if CONF_TIME_SYNC_MARKS in config:
        cg.add(var.set_time_sync_marks(config[CONF_TIME_SYNC_MARKS]))
    if (CONF_MCU_RELOAD_COUNTER) in config:
        sens = await sensor.new_sensor(config[CONF_MCU_RELOAD_COUNTER])
        cg.add(var.set_reset_counter(sens))
    
    if CONF_SHEDULE in config:
        shed = config[CONF_SHEDULE]
        if CONF_SHEDULE_HOURS in shed:
            conf = shed[CONF_SHEDULE_HOURS]
            numb = await number.new_number(conf, min_value=0, max_value=23, step=1)
            cg.add(var.set_hours_number(numb))
        if CONF_SHEDULE_MINUTES in shed:
            conf = shed[CONF_SHEDULE_MINUTES]
            numb = await number.new_number(conf, min_value=0, max_value=59, step=1)
            cg.add(var.set_minutes_number(numb))
        if CONF_SHEDULE_TEMPERATURE in shed:
            conf = shed[CONF_SHEDULE_TEMPERATURE]
            #numb = await number.new_number(conf, min_value=visual[CONF_MIN_TEMPERATURE], max_value=visual[CONF_MAX_TEMPERATURE], step=visual[CONF_TEMPERATURE_STEP])
            numb = await number.new_number(conf, min_value=visual[CONF_MIN_TEMPERATURE], max_value=visual[CONF_MAX_TEMPERATURE], step=0.5)
            cg.add(var.set_temperatures_number(numb))
        if CONF_SHEDULE_SELECTOR in shed:
            conf = shed[CONF_SHEDULE_SELECTOR]
            sel = await select.new_select(conf, options=[])
            cg.add(var.set_plan_select(sel))

