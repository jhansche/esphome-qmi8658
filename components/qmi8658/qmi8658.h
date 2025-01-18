#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

#include "qmi8658_types.h"

namespace esphome {
namespace qmi8658 {

typedef struct __IMUdata {
  float x;
  float y;
  float z;
} IMUdata;

class QMI8658Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;

  float get_setup_priority() const override;

  void set_accel_range(QMI8658_AccRange accel_range) {
    accel_range_ = accel_range;

    switch (accel_range) {
      case QMI8658AccRange_2g:
        this->acc_lsb_div = 1 << 14;
        break;
      case QMI8658AccRange_4g:
        this->acc_lsb_div = 1 << 13;
        break;
      case QMI8658AccRange_8g:
        this->acc_lsb_div = 1 << 12;
        break;
      case QMI8658AccRange_16g:
        this->acc_lsb_div = 1 << 11;
        break;
      default:
        this->acc_lsb_div = 1 << 12;
        this->accel_range_ = QMI8658AccRange_8g;
        break;
    }
  }
  void set_accel_odr(QMI8658_AccOdr accel_odr) { accel_odr_ = accel_odr; }
  void set_accel_lpf_mode(bool accel_lpf_mode) {
    // accel_lpf_mode_ = accel_lpf_mode ? A_LSP_MODE_3 : LPF_DISABLED;
  }

  void set_gyro_range(QMI8658_GyrRange gyro_range) {
    gyro_range_ = gyro_range;
    switch (gyro_range) {
      case QMI8658GyrRange_16dps:
        this->gyro_lsb_div = 1024;
        break;
      case QMI8658GyrRange_32dps:
        this->gyro_lsb_div = 512;
        break;
      case QMI8658GyrRange_64dps:
        this->gyro_lsb_div = 256;
        break;
      case QMI8658GyrRange_128dps:
        this->gyro_lsb_div = 128;
        break;
      case QMI8658GyrRange_256dps:
        this->gyro_lsb_div = 64;
        break;
      case QMI8658GyrRange_512dps:
        this->gyro_lsb_div = 32;
        break;
      case QMI8658GyrRange_1024dps:
        this->gyro_lsb_div = 16;
        break;
      case QMI8658GyrRange_2048dps:
        this->gyro_lsb_div = 8;
        break;
      default:
        this->gyro_range_ = QMI8658GyrRange_512dps;
        this->gyro_lsb_div = 64;
        break;
    }
  }
  void set_gyro_odr(QMI8658_GyrOdr gyro_odr) { gyro_odr_ = gyro_odr; }
  void set_gyro_lpf_mode(bool lpf_enable) {
    // gyro_lpf_mode_ = lpf_enable ? G_LSP_MODE_3 : LPF_DISABLED;
  }

  void set_interrupt_pin_1(InternalGPIOPin *interrupt_pin) { interrupt_pin_1_ = interrupt_pin; }
  void set_interrupt_pin_2(InternalGPIOPin *interrupt_pin) { interrupt_pin_2_ = interrupt_pin; }
  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

 protected:
  void read_accelerometer();
  void read_gyro();
  QMI8658_AccRange accel_range_;
  QMI8658_AccOdr accel_odr_;
  QMI8658_LpfMode accel_lpf_mode_;
  bool accel_en{true};
  bool gyro_en{true};

  bool has_accel_() { return accel_x_sensor_ != nullptr || accel_y_sensor_ != nullptr || accel_z_sensor_ != nullptr; }
  bool has_gyro_() { return gyro_x_sensor_ != nullptr || gyro_y_sensor_ != nullptr || gyro_z_sensor_ != nullptr; }

  uint16_t acc_lsb_div = 0;
  uint16_t gyro_lsb_div = 0;

  QMI8658_GyrRange gyro_range_;
  QMI8658_GyrOdr gyro_odr_;
  QMI8658_LpfMode gyro_lpf_mode_;

  InternalGPIOPin *interrupt_pin_1_{nullptr};
  InternalGPIOPin *interrupt_pin_2_{nullptr};
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  IMUdata accel_data{};
  IMUdata gyro_data{};
  static void interrupt_(QMI8658Component *args);
  void configure_accelerometer_(QMI8658_AccRange range, QMI8658_AccOdr odr, QMI8658_LpfMode lpf_mode = LSP_MODE_0,
                                bool lpf_en = false);
  void configure_gyro_(QMI8658_GyrRange range, QMI8658_GyrOdr odr, QMI8658_LpfMode lpf_mode = LSP_MODE_0,
                       bool lpf_en = false);
  void enable_sensors_(bool accel_en, bool gyro_en);
};

}  // namespace qmi8658
}  // namespace esphome
