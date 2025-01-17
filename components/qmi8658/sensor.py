import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ADDRESS,
    CONF_ACCELERATION_X,
    CONF_ACCELERATION_Y,
    CONF_ACCELERATION_Z,
    CONF_GYROSCOPE_X,
    CONF_GYROSCOPE_Y,
    CONF_GYROSCOPE_Z,
    CONF_INTERRUPT_PIN,
    CONF_TEMPERATURE,
    CONF_ID,
    UNIT_DEGREE_PER_SECOND,
    UNIT_G,
    DEVICE_CLASS_TEMPERATURE,
    ICON_SCREEN_ROTATION,
    ICON_BRIEFCASE_DOWNLOAD,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from esphome import pins

CONF_INTERRUPT_GROUP = "interrupt_group"
CONF_INTERRUPT_PIN_1 = CONF_INTERRUPT_PIN + "_1"
CONF_INTERRUPT_PIN_2 = CONF_INTERRUPT_PIN + "_2"

CONF_ACCELERATION_RANGE = "acceleration_range"
CONF_ACCELERATION_ODR = "acceleration_odr"
CONF_ACCELERATION_LPF_MODE = "acceleration_lpf_mode"
CONF_GYROSCOPE_RANGE = "gyroscope_range"
CONF_GYROSCOPE_ODR = "gyroscope_odr"
CONF_GYROSCOPE_LPF_MODE = "gyroscope_lpf_mode"

DEPENDENCIES = ["i2c"]

qmi8658_ns = cg.esphome_ns.namespace("qmi8658")


QMI8658Component = qmi8658_ns.class_(
    "QMI8658Component", cg.PollingComponent, i2c.I2CDevice
)

QMI8658AccelRange = qmi8658_ns.enum("QMI8658_AccRange")
QMI8658AccelRanges = {
    "2G": QMI8658AccelRange.QMI8658AccRange_2g,
    "4G": QMI8658AccelRange.QMI8658AccRange_4g,
    "8G": QMI8658AccelRange.QMI8658AccRange_8g,
    "16G": QMI8658AccelRange.QMI8658AccRange_16g,
}

QMI8658AccelODR = qmi8658_ns.enum("QMI8658_AccOdr")
QMI8658AccelODRs = {
    "8KHZ": QMI8658AccelODR.QMI8658AccOdr_8000Hz,
    "4KHZ": QMI8658AccelODR.QMI8658AccOdr_4000Hz,
    "2KHZ": QMI8658AccelODR.QMI8658AccOdr_2000Hz,
    "1KHZ": QMI8658AccelODR.QMI8658AccOdr_1000Hz,
    "500HZ": QMI8658AccelODR.QMI8658AccOdr_500Hz,
    "250HZ": QMI8658AccelODR.QMI8658AccOdr_250Hz,
    "125HZ": QMI8658AccelODR.QMI8658AccOdr_125Hz,
    "62.5HZ": QMI8658AccelODR.QMI8658AccOdr_62_5Hz,
    "31.25HZ": QMI8658AccelODR.QMI8658AccOdr_31_25Hz,
    "LP_128HZ": QMI8658AccelODR.QMI8658AccOdr_LowPower_128Hz,
    "LP_21HZ": QMI8658AccelODR.QMI8658AccOdr_LowPower_21Hz,
    "LP_11HZ": QMI8658AccelODR.QMI8658AccOdr_LowPower_11Hz,
    "LP_3HZ": QMI8658AccelODR.QMI8658AccOdr_LowPower_3Hz,
}

QMI8658GyroRange = qmi8658_ns.enum("QMI8658_GyrRange")
QMI8658GyroRanges = {
    "32DPS": QMI8658GyroRange.QMI8658GyrRange_32dps,
    "64DPS": QMI8658GyroRange.QMI8658GyrRange_64dps,
    "128DPS": QMI8658GyroRange.QMI8658GyrRange_128dps,
    "256DPS": QMI8658GyroRange.QMI8658GyrRange_256dps,
    "512DPS": QMI8658GyroRange.QMI8658GyrRange_512dps,
    "1024DPS": QMI8658GyroRange.QMI8658GyrRange_1024dps,
    "2048DPS": QMI8658GyroRange.QMI8658GyrRange_2048dps,
    "4096DPS": QMI8658GyroRange.QMI8658GyrRange_4096dps,
}

QMI8658GyroODR = qmi8658_ns.enum("QMI8658_GyrOdr")
QMI8658GyroODRs = {
    "8KHZ": QMI8658GyroODR.QMI8658GyrOdr_8000Hz,
    "4KHZ": QMI8658GyroODR.QMI8658GyrOdr_4000Hz,
    "2KHZ": QMI8658GyroODR.QMI8658GyrOdr_2000Hz,
    "1KHZ": QMI8658GyroODR.QMI8658GyrOdr_1000Hz,
    "500HZ": QMI8658GyroODR.QMI8658GyrOdr_500Hz,
    "250HZ": QMI8658GyroODR.QMI8658GyrOdr_250Hz,
    "125HZ": QMI8658GyroODR.QMI8658GyrOdr_125Hz,
    "62.5HZ": QMI8658GyroODR.QMI8658GyrOdr_62_5Hz,
    "31.25HZ": QMI8658GyroODR.QMI8658GyrOdr_31_25Hz,
}

QMI8658LpfMode = qmi8658_ns.enum("QMI8658_LpfModes")
QMI8658LpfModes = {
    "0": QMI8658LpfMode.QMI8658Lpf_Mode0,
    "1": QMI8658LpfMode.QMI8658Lpf_Mode1,
    "2": QMI8658LpfMode.QMI8658Lpf_Mode2,
    "3": QMI8658LpfMode.QMI8658Lpf_Mode3,
}


def validate_enum(enum_values, units=None, int=True):
    _units = []
    if units is not None:
        _units = units if isinstance(units, list) else [units]
        _units = [str(x) for x in _units]
    enum_bound = cv.enum(enum_values, int=int)

    def validate_enum_bound(value):
        value = cv.string(value)
        for unit in _units:
            if value.endswith(unit):
                value = value[: -len(unit)]
                break
        return enum_bound(value)

    return validate_enum_bound


acceleration_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_G,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)
gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QMI8658Component),
            cv.Optional(CONF_ADDRESS): cv.i2c_address,
            cv.Exclusive(
                CONF_INTERRUPT_PIN_1, CONF_INTERRUPT_GROUP
            ): pins.gpio_input_pin_schema,
            cv.Exclusive(
                CONF_INTERRUPT_PIN_2, CONF_INTERRUPT_GROUP
            ): pins.gpio_input_pin_schema,
            cv.Optional(CONF_ACCELERATION_RANGE, default="8G"): cv.one_of(
                *QMI8658AccelRanges,
                upper=True,
            ),
            cv.Optional(CONF_ACCELERATION_ODR, default="1KHZ"): cv.one_of(
                *QMI8658AccelODRs,
                upper=True,
            ),
            cv.Optional(CONF_ACCELERATION_LPF_MODE, default="true"): cv.boolean,
            cv.Optional(CONF_GYROSCOPE_RANGE, default="512DPS"): cv.one_of(
                *QMI8658GyroRanges,
                upper=True,
            ),
            cv.Optional(CONF_GYROSCOPE_ODR, default="1KHZ"): cv.one_of(
                *QMI8658GyroODRs,
                upper=True,
            ),
            cv.Optional(CONF_GYROSCOPE_LPF_MODE, default="true"): cv.boolean,
            cv.Optional(CONF_ACCELERATION_X): acceleration_schema,
            cv.Optional(CONF_ACCELERATION_Y): acceleration_schema,
            cv.Optional(CONF_ACCELERATION_Z): acceleration_schema,
            cv.Optional(CONF_GYROSCOPE_X): gyro_schema,
            cv.Optional(CONF_GYROSCOPE_Y): gyro_schema,
            cv.Optional(CONF_GYROSCOPE_Z): gyro_schema,
            cv.Optional(CONF_TEMPERATURE): temperature_schema,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(i2c.i2c_device_schema(0x6B)),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    if CONF_INTERRUPT_PIN_1 in config:
        interrupt_pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN_1])
        cg.add(var.set_interrupt_pin_1(interrupt_pin))
    if CONF_INTERRUPT_PIN_2 in config:
        interrupt_pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN_2])
        cg.add(var.set_interrupt_pin_2(interrupt_pin))

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_accel_range(QMI8658AccelRanges[config[CONF_ACCELERATION_RANGE]]))
    cg.add(var.set_accel_odr(QMI8658AccelODRs[config[CONF_ACCELERATION_ODR]]))
    cg.add(var.set_accel_lpf_mode(config[CONF_ACCELERATION_LPF_MODE]))

    cg.add(var.set_gyro_range(QMI8658GyroRanges[config[CONF_GYROSCOPE_RANGE]]))
    cg.add(var.set_gyro_odr(QMI8658GyroODRs[config[CONF_GYROSCOPE_ODR]]))
    cg.add(var.set_gyro_lpf_mode(config[CONF_GYROSCOPE_LPF_MODE]))

    if CONF_ACCELERATION_X in config:
        sens = await sensor.new_sensor(config[CONF_ACCELERATION_X])
        cg.add(var.set_accel_x_sensor(sens))
    if CONF_ACCELERATION_Y in config:
        sens = await sensor.new_sensor(config[CONF_ACCELERATION_Y])
        cg.add(var.set_accel_y_sensor(sens))
    if CONF_ACCELERATION_Z in config:
        sens = await sensor.new_sensor(config[CONF_ACCELERATION_Z])
        cg.add(var.set_accel_z_sensor(sens))
    if CONF_GYROSCOPE_X in config:
        sens = await sensor.new_sensor(config[CONF_GYROSCOPE_X])
        cg.add(var.set_gyro_x_sensor(sens))
    if CONF_GYROSCOPE_Y in config:
        sens = await sensor.new_sensor(config[CONF_GYROSCOPE_Y])
        cg.add(var.set_gyro_y_sensor(sens))
    if CONF_GYROSCOPE_Z in config:
        sens = await sensor.new_sensor(config[CONF_GYROSCOPE_Z])
        cg.add(var.set_gyro_z_sensor(sens))
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
