#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

#include "qmi8658_types.h"

namespace esphome {
namespace qmi8658 {

class QMI8658Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;

  float get_setup_priority() const override;

  void set_accel_range(QMI8658_AccRange accel_range) { accel_range_ = accel_range; }
  void set_accel_odr(QMI8658_AccOdr accel_odr) { accel_odr_ = accel_odr; }
  void set_accel_lpf_mode(bool accel_lpf_mode) { accel_lpf_mode_ = accel_lpf_mode ? A_LSP_MODE_3 : DISABLED; }

  void set_gyro_range(QMI8658_GyrRange gyro_range) { gyro_range_ = gyro_range; }
  void set_gyro_odr(QMI8658_GyrOdr gyro_odr) { gyro_odr_ = gyro_odr; }
  void set_gyro_lpf_mode(bool lpf_enable) { gyro_lpf_mode_ = lpf_enable ? G_LSP_MODE_3 : DISABLED; }

  void set_interrupt_pin_1(GPIOPin *interrupt_pin) { interrupt_pin_1_ = interrupt_pin; }
  void set_interrupt_pin_2(GPIOPin *interrupt_pin) { interrupt_pin_2_ = interrupt_pin; }
  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

 protected:
  QMI8658_AccRange accel_range_;
  QMI8658_AccOdr accel_odr_;
  QMI8658_LpfMode accel_lpf_mode_;

  QMI8658_GyrRange gyro_range_;
  QMI8658_GyrOdr gyro_odr_;
  QMI8658_LpfMode gyro_lpf_mode_;

  GPIOPin *interrupt_pin_1_{nullptr};
  GPIOPin *interrupt_pin_2_{nullptr};
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  IMUdata accel_data;
  IMUdata gyro_data;
};
typedef struct IMUdata {
  /* data */
};

}  // namespace qmi8658
}  // namespace esphome
